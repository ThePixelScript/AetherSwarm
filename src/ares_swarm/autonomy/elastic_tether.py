"""Read-only per-link motion governor; never commits state or replaces safety."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

from ..core.enums import FailureState, RTHState
from ..core.kinematics import move_towards
from ..energy.battery import calculate_energy_cost
from ..safety.separation import min_continuous_separation


@dataclass(frozen=True)
class TetherConfig:
    safe_hop_m: float = 85.0
    margin_m: float = 5.0

    def __post_init__(self):
        if not (math.isfinite(self.safe_hop_m) and math.isfinite(self.margin_m)
                and 0 <= self.margin_m < self.safe_hop_m):
            raise ValueError('Require finite 0 <= margin < safe hop')

    @property
    def limit(self):
        return self.safe_hop_m - self.margin_m


class ElasticChainTether:
    """Shorten nominal steps, then verify the safety-filtered batch with Gamma.

    Local step shortening preserves geofence-valid path prefixes and explicitly
    rechecks continuous separation. RTH is excluded from chain membership, but
    may still wait for separation or protected-peer constraints.
    This is a conservative governor, not a connectivity-restoration controller.
    """
    def __init__(self, relay_manager, analyzer, config=None):
        self.manager = relay_manager
        self.analyzer = analyzer
        self.config = config or TetherConfig()
        self.reset()

    def reset(self):
        self.holds = []
        self.gamma_checks = 0
        self.post_safety_scales = 0

    def links(self, snapshot):
        pairs = set()
        for chain in sorted(self.manager.chains.values(), key=lambda c: c.chain_id):
            if chain.status == 'TEARDOWN':
                continue
            upstream = None  # GCS
            for uid in [*chain.relay_ids, chain.surveyor_id]:
                u = snapshot.uavs.get(uid)
                if not u or not u.active or u.failure_state == FailureState.FAILED or u.rth_state != RTHState.NONE:
                    break
                pairs.add((upstream, uid))
                upstream = uid
        return sorted(pairs, key=lambda p: (p[0] or '', p[1]))

    def prepare_snapshot(self, snapshot, speed, dt):
        """One-tick copied targets; authoritative long-range targets stay intact."""
        links = self.links(snapshot)
        positions = {uid: (move_towards(u.position_xy, u.target_position, speed, dt)[0]
                           if u.active and u.failure_state != FailureState.FAILED and u.target_position else u.position_xy)
                     for uid, u in snapshot.uavs.items()}
        uavs = dict(snapshot.uavs)
        for upstream, uid in links:
            u = snapshot.uavs[uid]
            anchor = snapshot.gcs_position if upstream is None else snapshot.uavs[upstream].position_xy
            limit = max(self.config.limit, math.dist(u.position_xy, anchor))
            end = positions[uid]
            if math.dist(end, anchor) <= limit + 1e-9:
                continue
            # Circle intersection along this exact nominal segment (convex).
            lo, hi = 0., 1.
            for _ in range(45):
                a = (lo + hi) / 2
                point = tuple(p + a * (q-p) for p, q in zip(u.position_xy, end))
                if math.dist(point, anchor) <= limit:
                    lo = a
                else:
                    hi = a
            target = tuple(p + lo * (q-p) for p, q in zip(u.position_xy, end))
            positions[uid] = target
            uavs[uid] = replace(u, target_position=target)
            self.holds.append({'tick': snapshot.simulation_tick, 'uav': uid, 'upstream': upstream or 'gcs', 'fraction': lo})
        return replace(snapshot, uavs=uavs)

    def filter_commands(self, snapshot, commands, dt, idle_rate, movement_rate):
        """Verify actual safety-filtered positions, not optimistic nominal motion."""
        links = self.links(snapshot)
        if not links:
            return commands
        protected = set(self.analyzer.analyze(snapshot).connected_uav_ids)
        # A returning node may legitimately leave the network; never tether RTH.
        protected = {uid for uid in protected if snapshot.uavs[uid].rth_state == RTHState.NONE}

        def candidate(scales):
            uavs = dict(snapshot.uavs)
            output = []
            for cmd in commands:
                u = snapshot.uavs[cmd.uav_id]
                alpha = scales[cmd.uav_id]
                point = tuple(p + alpha*(q-p) for p,q in zip(u.position_xy, cmd.new_position_xy))
                output.append(replace(cmd, new_position_xy=point,
                    new_velocity_xy=tuple(alpha*v for v in cmd.new_velocity_xy),
                    delta_energy=calculate_energy_cost(dt=dt, distance=math.dist(u.position_xy, point), idle_rate=idle_rate, movement_rate=movement_rate)))
                uavs[u.id] = replace(u, position_xy=point)
            return replace(snapshot, uavs=uavs), output

        scales = {cmd.uav_id: 1. for cmd in commands}
        for attempt in range(65):
            prospective, output = candidate(scales)
            reduce_ids = set()
            for upstream, uid in links:
                old_anchor = snapshot.gcs_position if upstream is None else snapshot.uavs[upstream].position_xy
                anchor = prospective.gcs_position if upstream is None else prospective.uavs[upstream].position_xy
                allowed = max(self.config.limit, math.dist(snapshot.uavs[uid].position_xy, old_anchor))
                if math.dist(prospective.uavs[uid].position_xy, anchor) > allowed + 1e-8:
                    # Stop only endpoints whose advance stretches this link.
                    for endpoint, other_old, other_new in (
                        (uid, old_anchor, anchor),
                        (upstream, snapshot.uavs[uid].position_xy, prospective.uavs[uid].position_xy)):
                        if endpoint in scales:
                            old = snapshot.uavs[endpoint].position_xy
                            new = prospective.uavs[endpoint].position_xy
                            if math.dist(new, other_new) > math.dist(old, other_new) + 1e-10:
                                reduce_ids.add(endpoint)
                    if not reduce_ids:
                        reduce_ids.update(u for u in (upstream, uid) if u in scales)
            # Independent slowing is not automatically separation-safe. Verify
            # the whole continuous step again, including unrelated returning UAVs.
            ids = sorted(snapshot.uavs)
            for i, left in enumerate(ids):
                for right in ids[i+1:]:
                    a, b = snapshot.uavs[left], snapshot.uavs[right]
                    end_a, end_b = prospective.uavs[left], prospective.uavs[right]
                    va = tuple((q-p)/dt for p,q in zip(a.position_xy,end_a.position_xy))
                    vb = tuple((q-p)/dt for p,q in zip(b.position_xy,end_b.position_xy))
                    required = min(20.005, math.dist(a.position_xy,b.position_xy))
                    if min_continuous_separation(a.position_xy,va,b.position_xy,vb,dt) < required-1e-12:
                        reduce_ids.update(u for u in (left,right) if u in scales)
            if reduce_ids:
                for uid in reduce_ids:
                    scales[uid] = scales[uid]*.5 if attempt < 60 else 0.
                continue
            self.gamma_checks += 1
            if protected <= set(self.analyzer.analyze(prospective).connected_uav_ids):
                if any(a < 1 for a in scales.values()):
                    self.post_safety_scales += 1
                return output
            scales = {uid: a*.5 if attempt < 60 else 0. for uid,a in scales.items()}
        raise RuntimeError('Unchanged snapshot failed deterministic Gamma validation')
