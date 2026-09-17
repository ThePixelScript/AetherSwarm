# M0 graph, connectivity and shared analysis

Owner: Shakeel. Existing StateStore, UAVState, events, clock and shared core state
are reused without modification.

## Pipeline

    StateSnapshot
      -> ChannelModel
      -> build_network_graph (NetworkX Graph)
      -> analyze_connectivity + shortest_hop_routes
      -> existing interfaces.communication.NetworkAnalysis

BaselineCommunicationAnalyzer in communication/analysis.py is the small composition
adapter implementing the existing CommunicationAnalyzer.analyze(snapshot) protocol.
It receives only immutable snapshots, never StateStore or its writer capability.

Example consumer usage:

    analyzer = BaselineCommunicationAnalyzer(config.communication)
    result = analyzer.analyze(snapshot)
    route = result.routes_to_gcs["uav-2"]  # tuple UAV->...->GCS, or None
    hops = result.hop_counts["uav-2"]      # integer, or None

## Derived graph

Graph nodes are the GCS plus available UAVs. Inactive/failed/lost UAVs are excluded;
active disconnected UAVs remain as isolated or partitioned nodes.
The graph is simple, undirected and rebuilt on every call.

Nodes and unordered pairs are processed in lexical ID order. Each usable edge
contains the canonical LinkState under attribute link, and matching public metrics:
source_id, target_id, distance, rssi, snr, packet_loss_probability, estimated_pdr,
latency_ms, etx, link_quality, active and last_updated, plus range_feasible=True.

Edge absence means currently unusable, not necessarily physically out of range.
A degraded positive-PDR link remains an edge. A zero-PDR or outage link is absent.

nx.freeze blocks topology edits. NetworkX attribute dictionaries remain mutable
derived data; changing one cannot modify a snapshot or affect the next rebuild.
No mutable graph is exposed in NetworkAnalysis. Immutable LinkState objects are
shared safely. No cached or global graph is maintained.

## Connectivity semantics

One GCS BFS determines reachable UAVs and minimum hops.
Connected components and articulation points use standard NetworkX algorithms.
Every output collection is explicitly canonicalized:
- reachable/disconnected IDs sorted;
- each component sorted; components sorted lexicographically;
- articulation points sorted;
- maps inserted in sorted ID order.

Articulation points may include GCS. They describe the entire current graph,
including disconnected components. They do not assign RELAY roles or assert that
each articulation point disconnects a GCS-reachable UAV.

Failed/inactive UAVs are absent from both reachable and disconnected lists.
The health denominator is active UAVs, not originally deployed UAVs.

## One shared NetworkAnalysis

The existing class in interfaces/communication.py is extended, not duplicated.
Its first three constructor arguments remain unchanged:
snapshot_revision, network, connected_uav_ids.

Complete M0 results additionally expose:
- gcs_id and simulation_time;
- reachable_uav_ids: alias of connected_uav_ids;
- disconnected_uav_ids and components;
- routes_to_gcs: mapping for every active UAV, with None when disconnected;
- hop_counts: mapping for every active UAV, with None when disconnected;
- articulation_points;
- edge_metrics: alias of network.links (one source of canonical usable metrics);
- network_health: immutable mapping with the fields below.

Maps are recursively frozen; routes/components are tuples. Complete results validate
coverage, reciprocal reachability/component consistency, route endpoints, hops,
canonical usable edges and route edge existence.

Legacy partial instances/JSON remain supported with gcs_id=None. Consumers requiring
complete M0 information should use BaselineCommunicationAnalyzer output, not infer
that an empty legacy map proves disconnection. No core serialization registry
change is needed because the same existing class remains allowlisted.

NetworkState within the result is derived, not committed. Its recovery_state is
copied unchanged from the snapshot; communication never advances recovery.
Core UAV.connected_to_gcs and route fields are not overwritten by analysis.

## Network health definitions

| Key | Definition |
|---|---|
| active_uav_count | Number of UAV graph nodes, excluding GCS |
| connected_uav_count | Active UAVs with a GCS path |
| disconnected_uav_count | Active UAVs without a GCS path |
| connectivity_ratio | connected / active; None when active=0 |
| component_count | All components, including GCS component |
| disconnected_component_count | Components not containing GCS |
| largest_component_size | Maximum node count, including GCS if present |
| average_hop_count | Mean hops over connected UAVs; None if none connected |
| maximum_hop_count | Maximum connected UAV hops; None if none connected |
| articulation_point_count | All articulation points, including GCS if applicable |

These are instantaneous topology observations, not packet statistics, time-averaged
availability, downtime, recovery time or benchmark outcomes. No weak-link threshold
is invented for M0.

## Complexity

Let V include GCS, E be usable edges, and P the sum of output route lengths.
Pairwise channel evaluation is O(V^2); graph memory is O(V+E).
BFS, components and articulation analysis are O(V+E).
Canonical sorting adds O(V log V) for topology result IDs/components.
Assembling sorted edge metrics adds O(E log E); route outputs take O(P).
Complete-contract validation uses node/component lookups, avoiding per-edge scans
of all nodes. End-to-end memory includes O(P), which can be O(V^2) on a chain.

## Tests and integration boundaries

test_graph.py: GCS only, direct/two/three-hop chains, partitions, failed intermediate,
inactive nodes, edge attributes, conditions, repeated/reordered inputs, local graph
mutation isolation, and immutable source snapshots.
test_connectivity.py: connectivity/hops, multiple components, articulation including
GCS, no articulation, empty denominator, failure topology, determinism and bad graphs.
test_integration.py: GCS--u1--u2 followed by core MarkUAVFailed(u1), resulting u2
disconnection, no role/task/mission/store mutation from analysis, original YAML
loading, legacy constructor/JSON, output immutability, serialization and process
hash-seed independence.

No autonomy consumer is implemented or executed here. Sarath can review the additive
contract, Divesh can consume it, and Sujal can call it after producing each snapshot.
The plan's GCS-first example is adapted to the core's existing UAV-to-GCS route
direction. No simulation engine, state mutation, relay commands or safety policy is added.
