"""Unit tests for A0 baseline deterministic task allocator."""
from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Any

import pytest

from ares_swarm.autonomy.task_allocator import (
    A0TaskAllocator,
    AllocationResult,
    AllocationWeights,
    TaskActionProposal,
    TaskAllocatorConfig,
    TaskAssignment,
    UtilityScore,
)


@dataclass
class MockPosition:
    x: float
    y: float


@dataclass
class MockUAV:
    id: str
    position: MockPosition
    active: bool = True
    failure_status: str = "HEALTHY"
    role: str = "IDLE"
    assigned_task_id: str | None = None
    battery_pct: float | None = 100.0
    energy_remaining: float | None = 100.0
    estimated_rth_energy: float | None = 10.0
    safety_reserve: float | None = 10.0
    battery_capacity: float | None = 100.0
    role_lock_until: float = 0.0
    assignment_lock_until: float = 0.0
    cooldown_until: float = 0.0
    rth_state: str = "NONE"


@dataclass
class MockTask:
    id: str
    position: MockPosition
    priority: float = 1.0
    status: str = "PENDING"
    assigned_uav_id: str | None = None
    deadline: float | None = None
    created_time: float = 0.0
    is_emergency: bool = False


@dataclass
class MockSwarmState:
    uavs: tuple[MockUAV, ...]
    tasks: tuple[MockTask, ...]
    simulation_time: float = 0.0


@dataclass
class MockSnapshot:
    state: MockSwarmState
    revision: int = 1


# ---------------------------------------------------------------------------
# Test Cases
# ---------------------------------------------------------------------------

def test_highest_priority_feasible_task_preference():
    """Highest-priority feasible task must be preferred even when a lower-priority task is closer."""
    # UAV at (0, 0)
    uav = MockUAV(id="uav-1", position=MockPosition(0.0, 0.0))

    # Low-priority task very close at (10, 0)
    task_low = MockTask(id="task-low", position=MockPosition(10.0, 0.0), priority=1.0)
    # High-priority task further away at (100, 0)
    task_high = MockTask(id="task-high", position=MockPosition(100.0, 0.0), priority=10.0)

    allocator = A0TaskAllocator()
    result = allocator.allocate(uavs=[uav], tasks=[task_low, task_high])

    assert len(result.assignments) == 1
    assignment = result.assignments[0]
    assert assignment.uav_id == "uav-1"
    assert assignment.task_id == "task-high"
    assert "task-low" in result.unassigned_tasks


def test_infeasible_uav_exclusion():
    """Infeasible UAVs must be excluded from assignment with specific reasons."""
    # Candidate task
    task = MockTask(id="task-1", position=MockPosition(50.0, 50.0), priority=5.0)

    # Various infeasible UAVs
    uav_inactive = MockUAV(id="uav-inactive", position=MockPosition(0.0, 0.0), active=False)
    uav_failed = MockUAV(id="uav-failed", position=MockPosition(0.0, 0.0), failure_status="FAILED")
    uav_lost = MockUAV(id="uav-lost", position=MockPosition(0.0, 0.0), failure_status="LOST")
    uav_rth = MockUAV(id="uav-rth", position=MockPosition(0.0, 0.0), rth_state="RETURNING")
    uav_rth_role = MockUAV(id="uav-rth-role", position=MockPosition(0.0, 0.0), role="RETURN_TO_HOME")
    uav_relay = MockUAV(id="uav-relay", position=MockPosition(0.0, 0.0), role="RELAY")
    uav_locked = MockUAV(id="uav-locked", position=MockPosition(0.0, 0.0), cooldown_until=20.0)
    uav_assigned = MockUAV(id="uav-assigned", position=MockPosition(0.0, 0.0), assigned_task_id="prev-task")
    uav_low_batt = MockUAV(id="uav-low-batt", position=MockPosition(0.0, 0.0), battery_pct=10.0)  # threshold is 15%
    uav_no_energy = MockUAV(
        id="uav-no-energy",
        position=MockPosition(0.0, 0.0),
        energy_remaining=15.0,
        estimated_rth_energy=10.0,
        safety_reserve=10.0,  # 15 - 10 - 10 = -5 <= 0
    )

    infeasible_list = [
        uav_inactive,
        uav_failed,
        uav_lost,
        uav_rth,
        uav_rth_role,
        uav_relay,
        uav_locked,
        uav_assigned,
        uav_low_batt,
        uav_no_energy,
    ]

    allocator = A0TaskAllocator()
    result = allocator.allocate(uavs=infeasible_list, tasks=[task], simulation_time=5.0)

    # All UAVs should be rejected; no assignments made
    assert len(result.assignments) == 0
    assert "task-1" in result.unassigned_tasks
    assert len(result.infeasible_uavs) == len(infeasible_list)

    # Check reason messages
    assert "inactive" in result.infeasible_uavs["uav-inactive"]
    assert "FAILED" in result.infeasible_uavs["uav-failed"]
    assert "LOST" in result.infeasible_uavs["uav-lost"]
    assert "RETURNING" in result.infeasible_uavs["uav-rth"]
    assert "returning to home" in result.infeasible_uavs["uav-rth-role"]
    assert "eligible roles" in result.infeasible_uavs["uav-relay"]
    assert "locked or cooling down" in result.infeasible_uavs["uav-locked"]
    assert "already assigned" in result.infeasible_uavs["uav-assigned"]
    assert "below minimum" in result.infeasible_uavs["uav-low-batt"]
    assert "Insufficient energy" in result.infeasible_uavs["uav-no-energy"]


def test_infeasible_task_exclusion():
    """Completed, cancelled, or expired tasks must be excluded."""
    uav = MockUAV(id="uav-1", position=MockPosition(0.0, 0.0))

    task_completed = MockTask(id="task-comp", position=MockPosition(10.0, 0.0), status="COMPLETED")
    task_failed = MockTask(id="task-fail", position=MockPosition(10.0, 0.0), status="FAILED")
    task_in_progress = MockTask(id="task-prog", position=MockPosition(10.0, 0.0), status="IN_PROGRESS", assigned_uav_id="uav-x")
    task_expired = MockTask(id="task-exp", position=MockPosition(10.0, 0.0), deadline=10.0)

    allocator = A0TaskAllocator()
    result = allocator.allocate(
        uavs=[uav],
        tasks=[task_completed, task_failed, task_in_progress, task_expired],
        simulation_time=15.0,
    )

    assert len(result.assignments) == 0
    assert len(result.infeasible_tasks) == 4
    assert "uav-1" in result.unassigned_uavs


def test_deterministic_tie_breaking():
    """Ties between equidistant UAVs with identical state must break deterministically by ID."""
    task = MockTask(id="task-center", position=MockPosition(0.0, 0.0), priority=5.0)

    # 4 UAVs symmetrically positioned around task at distance 50m with identical battery
    uav_b = MockUAV(id="uav-b", position=MockPosition(50.0, 0.0), battery_pct=100.0)
    uav_a = MockUAV(id="uav-a", position=MockPosition(0.0, 50.0), battery_pct=100.0)
    uav_d = MockUAV(id="uav-d", position=MockPosition(-50.0, 0.0), battery_pct=100.0)
    uav_c = MockUAV(id="uav-c", position=MockPosition(0.0, -50.0), battery_pct=100.0)

    allocator = A0TaskAllocator()

    # Verify deterministic selection of 'uav-a' across 50 permutations of input order
    for _ in range(50):
        shuffled_uavs = [uav_b, uav_a, uav_d, uav_c]
        random.shuffle(shuffled_uavs)

        result = allocator.allocate(uavs=shuffled_uavs, tasks=[task])
        assert len(result.assignments) == 1
        assert result.assignments[0].uav_id == "uav-a", "Tie-breaking must pick 'uav-a' regardless of input permutation"
        assert result.assignments[0].task_id == "task-center"


def test_deterministic_task_tie_breaking():
    """Equal priority tasks must break deterministically by task ID."""
    uav = MockUAV(id="uav-1", position=MockPosition(0.0, 0.0))

    # Two tasks with identical priority at identical distance
    task_z = MockTask(id="task-z", position=MockPosition(10.0, 0.0), priority=2.0)
    task_a = MockTask(id="task-a", position=MockPosition(0.0, 10.0), priority=2.0)

    allocator = A0TaskAllocator()

    # Pass in [task_z, task_a]
    result1 = allocator.allocate(uavs=[uav], tasks=[task_z, task_a])
    assert result1.assignments[0].task_id == "task-a"

    # Pass in [task_a, task_z]
    result2 = allocator.allocate(uavs=[uav], tasks=[task_a, task_z])
    assert result2.assignments[0].task_id == "task-a"


def test_multiple_uav_task_assignment():
    """Multiple UAVs and tasks must be assigned deterministically."""
    uav1 = MockUAV(id="uav-1", position=MockPosition(0.0, 0.0))
    uav2 = MockUAV(id="uav-2", position=MockPosition(100.0, 100.0))
    uav3 = MockUAV(id="uav-3", position=MockPosition(500.0, 500.0))

    # Priority 3 near uav1, priority 2 near uav2, priority 1 near uav3
    task1 = MockTask(id="task-1", position=MockPosition(5.0, 5.0), priority=3.0)
    task2 = MockTask(id="task-2", position=MockPosition(105.0, 105.0), priority=2.0)
    task3 = MockTask(id="task-3", position=MockPosition(505.0, 505.0), priority=1.0)

    allocator = A0TaskAllocator()
    result = allocator.allocate(uavs=[uav1, uav2, uav3], tasks=[task1, task2, task3])

    assert len(result.assignments) == 3
    assert len(result.unassigned_tasks) == 0
    assert len(result.unassigned_uavs) == 0

    assigned_map = {a.task_id: a.uav_id for a in result.assignments}
    assert assigned_map["task-1"] == "uav-1"
    assert assigned_map["task-2"] == "uav-2"
    assert assigned_map["task-3"] == "uav-3"


def test_more_tasks_than_uavs():
    """When tasks outnumber UAVs, highest priority tasks are allocated and others remain unassigned."""
    uav1 = MockUAV(id="uav-1", position=MockPosition(0.0, 0.0))

    task1 = MockTask(id="task-p10", position=MockPosition(10.0, 0.0), priority=10.0)
    task2 = MockTask(id="task-p5", position=MockPosition(10.0, 0.0), priority=5.0)
    task3 = MockTask(id="task-p1", position=MockPosition(10.0, 0.0), priority=1.0)

    allocator = A0TaskAllocator()
    result = allocator.allocate(uavs=[uav1], tasks=[task1, task2, task3])

    assert len(result.assignments) == 1
    assert result.assignments[0].task_id == "task-p10"
    assert set(result.unassigned_tasks) == {"task-p5", "task-p1"}
    assert len(result.unassigned_uavs) == 0


def test_more_uavs_than_tasks():
    """When UAVs outnumber tasks, best UAV is allocated and others remain unassigned."""
    uav1 = MockUAV(id="uav-close", position=MockPosition(10.0, 0.0))
    uav2 = MockUAV(id="uav-far", position=MockPosition(500.0, 0.0))
    task = MockTask(id="task-1", position=MockPosition(0.0, 0.0), priority=2.0)

    allocator = A0TaskAllocator()
    result = allocator.allocate(uavs=[uav1, uav2], tasks=[task])

    assert len(result.assignments) == 1
    assert result.assignments[0].uav_id == "uav-close"
    assert result.unassigned_uavs == ("uav-far",)
    assert len(result.unassigned_tasks) == 0


def test_empty_and_no_feasible_cases():
    """Empty inputs and all-infeasible scenarios must execute cleanly without exceptions."""
    allocator = A0TaskAllocator()

    # 1. Zero UAVs, Zero Tasks
    r1 = allocator.allocate(uavs=[], tasks=[])
    assert len(r1.assignments) == 0
    assert len(r1.unassigned_tasks) == 0
    assert len(r1.unassigned_uavs) == 0

    # 2. Zero UAVs, 2 Tasks
    task = MockTask(id="task-1", position=MockPosition(0.0, 0.0))
    r2 = allocator.allocate(uavs=[], tasks=[task])
    assert len(r2.assignments) == 0
    assert r2.unassigned_tasks == ("task-1",)

    # 3. 2 UAVs, Zero Tasks
    uav = MockUAV(id="uav-1", position=MockPosition(0.0, 0.0))
    r3 = allocator.allocate(uavs=[uav], tasks=[])
    assert len(r3.assignments) == 0
    assert r3.unassigned_uavs == ("uav-1",)


def test_utility_score_calculation_and_breakdown():
    """Utility score must adhere precisely to the specified formula."""
    weights = AllocationWeights(
        wP=2.0,
        wT=0.01,
        wE=1.0,
        wC=0.0,
        wN=0.0,
        wS=0.5,
    )
    allocator = A0TaskAllocator(config=TaskAllocatorConfig(weights=weights))

    uav = MockUAV(
        id="uav-1",
        position=MockPosition(0.0, 0.0),
        battery_pct=80.0,  # energy_risk = (100 - 80) / 100 = 0.2
        assigned_task_id="old-task",  # switching cost = 1.0
    )
    task = MockTask(
        id="task-new",
        position=MockPosition(30.0, 40.0),  # dist = 50.0m
        priority=4.0,
    )

    score = allocator.compute_utility(uav, task)

    # Expected breakdown:
    # priority_term = 2.0 * 4.0 = 8.0
    # travel_cost_term = 0.01 * 50.0 = 0.5
    # energy_risk_term = 1.0 * 0.2 = 0.2
    # connectivity_term = 0.0
    # network_risk_term = 0.0
    # switching_cost_term = 0.5 * 1.0 = 0.5
    # total = 8.0 - 0.5 - 0.2 + 0.0 - 0.0 - 0.5 = 6.8
    assert pytest.approx(score.priority_term, 1e-6) == 8.0
    assert pytest.approx(score.travel_cost_term, 1e-6) == 0.5
    assert pytest.approx(score.energy_risk_term, 1e-6) == 0.2
    assert score.connectivity_term == 0.0
    assert score.network_risk_term == 0.0
    assert pytest.approx(score.switching_cost_term, 1e-6) == 0.5
    assert pytest.approx(score.total, 1e-6) == 6.8


def test_action_proposal_generation():
    """AllocationResult must generate typed ActionProposals matching engine schema."""
    uav = MockUAV(id="uav-1", position=MockPosition(0.0, 0.0))
    task = MockTask(id="task-1", position=MockPosition(10.0, 0.0), priority=5.0)

    allocator = A0TaskAllocator()
    result = allocator.allocate(uavs=[uav], tasks=[task], snapshot_revision=42)
    proposals = result.to_action_proposals()

    assert len(proposals) == 1
    p = proposals[0]
    assert isinstance(p, TaskActionProposal)
    assert p.snapshot_revision == 42
    assert p.intent == "ASSIGN_TASK"
    assert p.target == "uav-1"
    assert p.source == "task_allocator"
    assert p.parameters["task_id"] == "task-1"
    assert "task assignment" in p.reason


def test_snapshot_and_plan_protocol_compatibility():
    """TaskAllocator must work seamlessly with StateSnapshot protocol objects."""
    uav = MockUAV(id="uav-1", position=MockPosition(0.0, 0.0))
    task = MockTask(id="task-1", position=MockPosition(10.0, 0.0), priority=5.0)

    state = MockSwarmState(uavs=(uav,), tasks=(task,), simulation_time=12.5)
    snapshot = MockSnapshot(state=state, revision=7)

    allocator = A0TaskAllocator()

    # 1. Via allocate(snapshot)
    res = allocator.allocate(snapshot)
    assert len(res.assignments) == 1
    assert res.snapshot_revision == 7
    assert res.assignments[0].uav_id == "uav-1"

    # 2. Via plan(snapshot)
    proposals = allocator.plan(snapshot)
    assert len(proposals) == 1
    assert proposals[0].snapshot_revision == 7
    assert proposals[0].target == "uav-1"
    assert proposals[0].parameters["task_id"] == "task-1"


def test_emergency_task_priority():
    """Emergency tasks take precedence over non-emergency tasks of the same priority."""
    uav = MockUAV(id="uav-1", position=MockPosition(0.0, 0.0))
    task_normal = MockTask(id="task-normal", position=MockPosition(10.0, 0.0), priority=5.0, is_emergency=False)
    task_emer = MockTask(id="task-emer", position=MockPosition(20.0, 0.0), priority=5.0, is_emergency=True)

    allocator = A0TaskAllocator()
    result = allocator.allocate(uavs=[uav], tasks=[task_normal, task_emer])

    assert len(result.assignments) == 1
    assert result.assignments[0].task_id == "task-emer"
    assert "task-normal" in result.unassigned_tasks


def test_position_coordinate_formats():
    """Positions supplied as (x, y) tuples or PositionProtocol objects must both be supported."""
    uav_tuple = MockUAV(id="uav-tuple", position=(10.0, 20.0))  # type: ignore[arg-type]
    task_tuple = MockTask(id="task-tuple", position=(10.0, 50.0))  # type: ignore[arg-type]

    allocator = A0TaskAllocator()
    result = allocator.allocate(uavs=[uav_tuple], tasks=[task_tuple])

    assert len(result.assignments) == 1
    assert result.assignments[0].uav_id == "uav-tuple"
    assert result.assignments[0].task_id == "task-tuple"
    # distance = 30m, priority = 1.0, travel_cost = 0.001 * 30 = 0.03
    # energy_risk = 0.1 * 0.0 = 0.0
    # score = 1.0 - 0.03 = 0.97
    assert pytest.approx(result.assignments[0].score, 1e-6) == 0.97


def test_reassignment_with_switching_cost():
    """When allow_reassignment=True, currently assigned UAVs can be reassigned with switching penalty."""
    config = TaskAllocatorConfig(
        allow_reassignment=True,
        weights=AllocationWeights(wP=10.0, wT=0.0, wE=0.0, wS=2.0),
    )
    allocator = A0TaskAllocator(config=config)

    # uav already assigned to 'task-old'
    uav = MockUAV(id="uav-1", position=MockPosition(0.0, 0.0), assigned_task_id="task-old")
    task_new = MockTask(id="task-new", position=MockPosition(0.0, 0.0), priority=5.0)

    result = allocator.allocate(uavs=[uav], tasks=[task_new])
    assert len(result.assignments) == 1
    assert result.assignments[0].task_id == "task-new"
    # Score should reflect switching penalty: 10 * 5.0 - 2.0 * 1.0 = 48.0
    assert pytest.approx(result.assignments[0].score, 1e-6) == 48.0
    assert result.assignments[0].score_breakdown.switching_cost_term == 2.0


def test_closer_uav_preferred_for_same_priority():
    """Among UAVs with identical status, the closer UAV achieves higher utility."""
    uav_close = MockUAV(id="uav-close", position=MockPosition(10.0, 0.0))
    uav_far = MockUAV(id="uav-far", position=MockPosition(100.0, 0.0))
    task = MockTask(id="task-1", position=MockPosition(0.0, 0.0), priority=5.0)

    allocator = A0TaskAllocator()
    result = allocator.allocate(uavs=[uav_far, uav_close], tasks=[task])

    assert len(result.assignments) == 1
    assert result.assignments[0].uav_id == "uav-close"
    assert "uav-far" in result.unassigned_uavs

