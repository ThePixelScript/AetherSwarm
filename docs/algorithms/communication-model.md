# M0 deterministic communication model

Owner: Shakeel. Scope: analysis only, using the Phase-1 models and configuration.
This is a Stage-1 mathematical abstraction, not validated wireless propagation.

## Inputs and API

ChannelModel accepts the existing CommunicationConfig. Call evaluate(source, target,
simulation_time=..., condition=...) using existing GCSState/UAVState endpoints.
There is no RNG, wall-clock read, event execution, state write, motion or radio library.

Parameters reuse the current YAML fields:
- max_range: R > 0, meters.
- base_latency: L >= 0, milliseconds.
- packet_loss: p in [0,1], the zero-distance base loss.

A frozen LinkCondition optionally specifies this pair's current quality_multiplier m
in [0,1], latency_penalty_ms B >= 0, and outage flag. Defaults: m=1, B=0, no outage.
This is a communication-local input, not a new shared event or scenario schema.

## Exact equations

For Cartesian positions (x_a,y_a), (x_b,y_b):

    d = hypot(x_a-x_b, y_a-y_b)
    range_feasible = d <= R
    q = m / (1 + d/R)
    loss = 1 - (1-p)*q
    PDR = 1 - loss
    latency_ms = L*(1 + d/R) + B
    ETX = 1/PDR   only when PDR > 0

A link is usable only when both nodes are available, it is in range, no outage is
requested and the computed PDR is positive. GCS is always available in the current
core model. UAV availability requires active=True and status neither FAILED nor LOST.
DEGRADED status alone adds no undocumented numeric penalty.

At d=0 with defaults: q=1, PDR=1-p, latency=L.
At d=R with defaults: q=0.5, PDR=(1-p)/2, latency=2L. The boundary is inclusive.
Outside R, the pair is unusable even though the smooth formula would remain positive.
m=0 or p=1 gives zero PDR and therefore no usable link.

For fixed parameters, quality/PDR decrease with distance; loss/ETX increase.
Latency is nondecreasing (constant when L=0). No artificial reliability threshold
or epsilon is added. IEEE-754 rounding can turn extremely tiny PDR into zero when
forming 1-loss; that pair is then unusable by the same rule. Overflow is rejected.

## LinkState compatibility adapter

Core LinkState requires finite ETX >= 1. It cannot truthfully represent 1/0.
We preserve core/models.py and return a local ChannelEvaluation:
- distance and range_feasible are available for every evaluated pair;
- usable pairs contain the original LinkState, with all canonical numeric metrics;
- unusable pairs have link=None, estimated_pdr=0, loss=1 and etx=None;
- unavailable_reason is inactive_endpoint, outside_range, outage or zero_pdr.

No infinity, arbitrary large ETX or misleading finite ETX is stored for zero-PDR
pairs. Detailed latency/quality metrics are exposed through .link for usable pairs;
unusable pairs have no such LinkState. RSSI and SNR remain None: no invented dBm/dB.

The LinkState source_id/target_id are the sorted endpoint IDs, representing an
undirected pair in this derived result. last_updated is exactly simulation_time.

## Outage/degradation boundary

Graph construction and BaselineCommunicationAnalyzer accept explicit mappings from
unordered endpoint pairs to LinkCondition. Reversed duplicate keys and unknown node
references are rejected. Conditions are copied and frozen in the analyzer.

No scheduler is added. Sujal/Sarath can supply a new condition set when a future event
has been executed. Record these inputs alongside config/snapshot for reproducibility.
An unchanged analyzer has unchanged conditions at later times: it does not invent
onset, duration or restoration rules.

Snapshot.network.links contains stored observations, not a defined impairment
contract. It is deliberately not interpreted as an outage schedule or reused as
model input. This avoids feeding stale derived links back into the next analysis.

## Determinism, complexity and limitations

Pair evaluation is O(1) time/space. Reversing endpoints gives the same canonical link.
No seed is consumed; results are deterministic for the same snapshot/config/conditions.
Pin NetworkX/NumPy dependencies for reproducible installations.

The abstraction is homogeneous, reciprocal, 2D and obstacle-free. Altitude layers,
terrain, fading, contention, throughput, acknowledgments and retransmission delays
are not modeled. PDR is an estimate, not an observed delivered-packet ratio; latency
is a deterministic link estimate. ETX=1/PDR uses the requested single-delivery
convention, not a forward-times-reverse ACK delivery model.

## Tests

tests/communication/test_channel.py covers zero distance, range boundary/just outside,
inactive/FAILED/LOST endpoints, DEGRADED status, loss/PDR extremes, ETX, degradation,
outage, monotonicity, endpoint symmetry, repeated evaluation and invalid inputs.
Graph tests verify impaired and zero-PDR links are handled consistently.

Later model changes must preserve this baseline or introduce an explicit variant.
