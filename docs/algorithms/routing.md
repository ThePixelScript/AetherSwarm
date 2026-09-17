# M0 deterministic shortest-hop routing

Owner: Shakeel. Input: simple undirected NetworkX graph with string node IDs and GCS.
Output: immutable routes_to_gcs mapping for every active UAV. Disconnected UAVs map
to None; excluded failed/inactive nodes have no entry. GCS has no route-map entry.

## Algorithm

1. Compute unweighted distance-to-GCS h(v) using one BFS.
2. For every reachable v other than GCS, choose the lexicographically smallest
   neighbor n such that h(n)=h(v)-1.
3. Follow these next hops from each source until GCS.
4. Insert results in sorted source-ID order.

Routes are (source UAV, ..., GCS), matching existing core state conventions.

## Tie-breaking proof

All minimum-hop paths must begin with a neighbor one level closer to GCS.
The smallest such neighbor minimizes the first differing element of the source path.
Apply the same rule at each following node to minimize the remaining suffix.
Distances strictly decrease, preventing loops and guaranteeing exactly h(v) hops.

Thus the result is the lexicographically smallest complete minimum-hop path in the
UAV-to-GCS direction. Sorting a GCS-rooted BFS and reversing its paths is not used:
that can minimize the wrong end of a tied route.

No ETX, latency, quality, energy, or secondary reliability score affects M0 path
selection. A short poor link may be chosen over a longer high-quality path. That
is intentional for this baseline, not evidence of optimal resilience.

## Complexity

BFS and next-hop selection are O(V+E). Sorted source processing adds O(V log V).
Materializing all paths costs O(P), where P is total returned path length; a chain
has O(V^2) output size. No enumeration of all shortest paths is used in production.
Extra space beyond the input graph is O(V+P).

## Determinism and assumptions

No randomness, wall clock, history, global graph, or hidden cache is used.
The chosen neighbor is min(...) by string ID, independent of edge insertion order.
String lexicographic order is literal: e.g. u10 sorts before u2.
The graph is the usable-link graph from build_network_graph. Standalone functions
do not reinterpret edge metrics or filter externally supplied graph edges.
Directed graphs, multigraphs, missing GCS, invalid IDs and self-loops are rejected.

## Tests

tests/communication/test_routing.py covers:
- direct paths and minimum-hop multi-hop paths;
- all insertion permutations of a diamond;
- a tie that distinguishes source-first from reversed GCS-first ordering;
- ignoring edge weights in M0;
- disconnected and GCS-only graphs;
- failed relay exclusion;
- repeated immutable results;
- comparison with the lexicographically minimum of all shortest paths for all
  64 simple undirected graphs on four labeled nodes.

Integration tests also compare serialized analyses across independently launched
Python processes with three different PYTHONHASHSEED values.

## Limitations and next milestone boundary

No retries, queueing, packet forwarding, measured PDR, failover timing, adaptive
relay, recovery plan or handover execution is implemented. Graph rebuilds merely
report paths after the caller supplies a changed snapshot/condition.

Keep this baseline intact for later comparisons. M1 concerns communication-aware
consumer integration; reliability-weighted routing is a separate later milestone.
