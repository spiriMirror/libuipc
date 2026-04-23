#pragma once
#include <uipc/common/type_define.h>
#include <uipc/geometry/geometry.h>
#include <vector>

namespace uipc::geometry
{
/**
 * @brief Partition a general graph into clusters of at most `max_cluster_size`
 *        vertices using METIS_PartGraphKway.
 *
 * Equivalent to mesh_partition() but works on an arbitrary edge list rather
 * than a SimplicialComplex.  Falls back to sequential grouping when METIS is
 * unavailable or the graph has no edges.
 *
 * @param n_vertices      Total number of vertices (graph nodes).
 * @param edges           Undirected edges as (u, v) pairs (may contain duplicates;
 *                        will be deduplicated internally).
 * @param max_cluster_size Maximum vertices per cluster (e.g. BANKSIZE/4 = 4 for ABD).
 * @return                Vector of length n_vertices; element i is the partition
 *                        ID (0-indexed, contiguous) for vertex i.
 */
UIPC_GEOMETRY_API std::vector<IndexT> graph_partition(SizeT                        n_vertices,
                                                       const std::vector<Vector2i>& edges,
                                                       SizeT max_cluster_size);

/**
 * @brief Weighted overload of graph_partition.
 *
 * Each edge in `edges` has a corresponding weight in `edge_weights` (same length).
 * METIS uses these weights as `adjwgt` to bias partitioning: higher-weight edges
 * are preferred to remain intra-cluster (less likely to be cut).
 *
 * Useful when some edges represent "must-not-split" groups (e.g. ABD body cliques
 * where the 4 sub-nodes of one body must live in the same cluster).
 *
 * @param n_vertices      Total number of vertices.
 * @param edges           Undirected edges as (u, v) pairs.
 * @param edge_weights    Per-edge weights (same length as `edges`). All must be > 0.
 * @param max_cluster_size Maximum vertices per cluster.
 */
UIPC_GEOMETRY_API std::vector<IndexT> graph_partition(SizeT                        n_vertices,
                                                       const std::vector<Vector2i>& edges,
                                                       const std::vector<IndexT>&   edge_weights,
                                                       SizeT max_cluster_size);
}  // namespace uipc::geometry
