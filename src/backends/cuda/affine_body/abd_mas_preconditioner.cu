#include <type_define.h>
#include <linear_system/local_preconditioner.h>
#include <affine_body/abd_linear_subsystem.h>
#include <affine_body/inter_affine_body_constitution_manager.h>
#include <linear_system/global_linear_system.h>
#include <finite_element/mas_preconditioner_engine.h>
#include <sim_engine.h>
#include <backends/common/backend_path_tool.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/log.h>
#include <uipc/geometry/utils/graph_partition.h>
#include <set>
#include <fstream>
#include <filesystem>

namespace uipc::backend::cuda
{
// Forward declaration from fem_mas_preconditioner.cu — reuse the same helper.
void fill_identity_indices(muda::DeviceBuffer<uint32_t>& buf, int count);

/**
 * @brief ABD MAS (MultiLevel Additive Schwarz) Preconditioner
 *
 * Wraps MASPreconditionerEngine for the Affine Body Dynamics subsystem.
 *
 * Each affine body has 12 DOFs stored as 4 "nodes" × 3 DOFs/node in
 * the 3×3 BCOO:
 *   node 0 → translation t   (3 DOFs)
 *   node 1 → deformation A₁  (3 DOFs)
 *   node 2 → deformation A₂  (3 DOFs)
 *   node 3 → deformation A₃  (3 DOFs)
 *
 * Topology: we expand every body-level joint edge into 16 node-pair edges
 * (4 nodes of body i × 4 nodes of body j), plus 6 intra-body node-pair
 * edges per body.  METIS is then called at the node level with
 * max_cluster_size = BANKSIZE = 16 — topologically identical to FEM
 * mesh_partition(sc, 16).
 *
 * Activates automatically when InterAffineBodyConstitutionManager is present.
 * ABDDiagPreconditioner is disabled in that case.
 */
class ABDMASPreconditioner final : public LocalPreconditioner
{
  public:
    using LocalPreconditioner::LocalPreconditioner;

    static constexpr int BANKSIZE = MASPreconditionerEngine::BANKSIZE;

  private:
    ABDLinearSubsystem*                 abd_linear_subsystem                   = nullptr;
    InterAffineBodyConstitutionManager* inter_affine_body_constitution_manager = nullptr;

    MASPreconditionerEngine      m_engine;
    muda::DeviceBuffer<uint32_t> m_sorted_indices;

    int m_body_count    = 0;
    int m_active_levels = 0;

    // -------------------------------------------------------------------------
    // do_build
    // -------------------------------------------------------------------------
    // Wires the two required subsystems and reads optional config overrides.
    virtual void do_build(BuildInfo& info) override
    {
        abd_linear_subsystem = &require<ABDLinearSubsystem>();
        inter_affine_body_constitution_manager =
            &require<InterAffineBodyConstitutionManager>();

        info.connect(abd_linear_subsystem);

        auto lvl_attr =
            world().scene().config().find<IndexT>("extras/precond/abd_mas_active_levels");
        m_active_levels = lvl_attr ? static_cast<int>(lvl_attr->view()[0]) : 0;
        logger::info("ABDMASPreconditioner: active_levels config = {}", m_active_levels);
    }

    // -------------------------------------------------------------------------
    // do_init
    // -------------------------------------------------------------------------
    // Builds the node-level neighbor list and METIS partition, then initialises
    // the MAS engine.  Called once after the scene is fully set up.
    //
    // Step 1: download + deduplicate body-pair joint edges.
    //
    // Step 2: expand to node-pair edge list h_node_pairs:
    //   • Intra-body: all C(4,2)=6 pairs among the 4 nodes of each body.
    //   • Inter-body: all 4×4=16 node pairs for each joint (bi,bj).
    //   graph_partition deduplicates internally.
    //
    // Step 3: build symmetric CSR neighbor arrays from h_node_pairs
    //   (used by build_connect_mask_L0 inside the engine).
    //
    // Step 4: METIS node-level partition with max_cluster_size = BANKSIZE = 16.
    //   Identical to what FEM does with mesh_partition(sc, 16).
    //   METIS naturally keeps the 4 fully-connected nodes of each body together.
    //
    // Step 5: build part_to_real / real_to_part mappings and init the engine.
    virtual void do_init(InitInfo& info) override
    {
        m_body_count = static_cast<int>(abd_linear_subsystem->dof_count() / 12);
        UIPC_ASSERT(m_body_count > 0,
                    "ABDMASPreconditioner: no affine bodies found. "
                    "Body count must be > 0 when this preconditioner is active.");

        SizeT body_count = static_cast<SizeT>(m_body_count);
        SizeT node_count = body_count * 4;

        // ---- Step 1: Download + deduplicate inter-body joint edges ----
        auto d_edges = inter_affine_body_constitution_manager->inter_body_edges();
        std::vector<Vector2i> h_body_pairs(d_edges.size());
        d_edges.copy_to(h_body_pairs.data());

        for(auto& p : h_body_pairs)
            if(p[0] > p[1])
                std::swap(p[0], p[1]);

        std::ranges::sort(h_body_pairs,
                          [](const Vector2i& a, const Vector2i& b) {
                              return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
                          });
        {
            auto [new_end, _] = std::ranges::unique(h_body_pairs);
            h_body_pairs.erase(new_end, h_body_pairs.end());
        }

        // ---- Step 2: Expand body pairs → node-pair edge list with weights ----
        // Intra-body edges get a HUGE weight so METIS never cuts a body's K4 clique.
        // Without this, 10%+ of bodies get split across clusters, which destroys
        // the L0 block-Jacobi quality (ABDDiag's per-body 12x12 inverse strictly
        // dominates split-body L0).
        std::vector<Vector2i> h_node_pairs;
        std::vector<IndexT>   h_node_weights;
        h_node_pairs.reserve(body_count * 6 + h_body_pairs.size() * 16);
        h_node_weights.reserve(h_node_pairs.capacity());

        constexpr IndexT INTRA_BODY_WEIGHT = 100000;
        constexpr IndexT INTER_BODY_WEIGHT = 1;

        // intra-body: C(4,2)=6 edges per body, heavy weight
        for(SizeT b = 0; b < body_count; b++)
            for(int k = 0; k < 4; k++)
                for(int l = k + 1; l < 4; l++)
                {
                    h_node_pairs.push_back(
                        Vector2i(static_cast<int>(b * 4 + k),
                                 static_cast<int>(b * 4 + l)));
                    h_node_weights.push_back(INTRA_BODY_WEIGHT);
                }

        // inter-body: 16 node pairs per joint, unit weight
        for(auto& bp : h_body_pairs)
        {
            IndexT bi = bp[0], bj = bp[1];
            for(int k = 0; k < 4; k++)
                for(int l = 0; l < 4; l++)
                {
                    int na = static_cast<int>(bi * 4 + k);
                    int nb = static_cast<int>(bj * 4 + l);
                    h_node_pairs.push_back(
                        Vector2i(std::min(na, nb), std::max(na, nb)));
                    h_node_weights.push_back(INTER_BODY_WEIGHT);
                }
        }
        // graph_partition will deduplicate internally (summing duplicate weights)

        // ---- Step 3: Build symmetric CSR neighbor arrays ----
        std::vector<std::set<unsigned int>> node_neighbors(node_count);
        for(auto& e : h_node_pairs)
        {
            node_neighbors[e[0]].insert(static_cast<unsigned int>(e[1]));
            node_neighbors[e[1]].insert(static_cast<unsigned int>(e[0]));
        }

        std::vector<unsigned int> h_neighbor_list;
        std::vector<unsigned int> h_neighbor_start(node_count);
        std::vector<unsigned int> h_neighbor_num(node_count);
        for(SizeT i = 0; i < node_count; i++)
        {
            h_neighbor_start[i] = static_cast<unsigned int>(h_neighbor_list.size());
            h_neighbor_num[i]   = static_cast<unsigned int>(node_neighbors[i].size());
            for(auto n : node_neighbors[i])
                h_neighbor_list.push_back(n);
        }

        // ---- Step 4: Node-level METIS partition (same as FEM mesh_partition(sc,16)) ----
        auto part_ids_raw = uipc::geometry::graph_partition(
            node_count, h_node_pairs, h_node_weights, static_cast<SizeT>(BANKSIZE));

        std::vector<int> part_ids(node_count);
        for(SizeT i = 0; i < node_count; i++)
            part_ids[i] = static_cast<int>(part_ids_raw[i]);

        int max_part_id = *std::ranges::max_element(part_ids);

        std::vector<std::vector<int>> part_blocks(max_part_id + 1);
        for(SizeT i = 0; i < node_count; i++)
            part_blocks[part_ids[i]].push_back(static_cast<int>(i));

        for(SizeT b = 0; b < part_blocks.size(); b++)
        {
            UIPC_ASSERT(static_cast<int>(part_blocks[b].size()) <= BANKSIZE,
                        "ABD MAS: partition {} has {} nodes (max {})",
                        b, part_blocks[b].size(), BANKSIZE);
        }

        // ---- Step 5: Build part_to_real / real_to_part and init engine ----
        int part_map_size = 0;
        for(auto& block : part_blocks)
        {
            int padded = (static_cast<int>(block.size()) + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
            part_map_size += padded;
        }

        std::vector<int> h_part_to_real(part_map_size, -1);
        std::vector<int> h_real_to_part(node_count, -1);

        int offset = 0;
        for(auto& block : part_blocks)
        {
            for(SizeT i = 0; i < block.size(); i++)
            {
                int real_idx                    = block[i];
                h_part_to_real[offset + (int)i] = real_idx;
                h_real_to_part[real_idx]        = offset + static_cast<int>(i);
            }
            int padded = (static_cast<int>(block.size()) + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
            offset += padded;
        }

        for(SizeT i = 0; i < node_count; ++i)
        {
            UIPC_ASSERT(h_real_to_part[i] >= 0 && h_real_to_part[i] < part_map_size,
                        "ABD MAS: real_to_part[{}]={} out of range [0, {})",
                        i, h_real_to_part[i], part_map_size);
        }

        m_engine.init_neighbor(static_cast<int>(node_count),
                               static_cast<int>(h_neighbor_list.size()),
                               part_map_size,
                               h_neighbor_list,
                               h_neighbor_start,
                               h_neighbor_num,
                               h_part_to_real,
                               h_real_to_part);
        m_engine.init_matrix();
        if(m_active_levels > 0)
            m_engine.set_active_level_num(m_active_levels);

        logger::info(
            "ABDMASPreconditioner: {} bodies, {} nodes, {} clusters (BANKSIZE={}), {} joints",
            body_count, node_count, max_part_id + 1, BANKSIZE, h_body_pairs.size());

        // ---- Partition dump: verify body nodes are not split across clusters ----
        {
            backend::BackendPathTool path_tool{std::string{workspace()}};
            auto out_dir  = path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "debug");
            std::filesystem::create_directories(out_dir);
            std::string dump_path = (out_dir / "partition_map.txt").string();

            std::ofstream ofs(dump_path);
            ofs << "# ABD MAS partition map\n";
            ofs << "# bodies=" << body_count << "  nodes=" << node_count
                << "  clusters=" << (max_part_id + 1) << "  BANKSIZE=" << BANKSIZE << "\n";
            ofs << "# Format: cluster_id | [body:sub_node real_idx part_pos] ...\n\n";

            bool any_split = false;
            for(int ci = 0; ci <= max_part_id; ci++)
            {
                ofs << "cluster " << ci << " (" << part_blocks[ci].size() << " nodes): ";
                for(int real_idx : part_blocks[ci])
                {
                    int body_idx = real_idx / 4;
                    int sub_node = real_idx % 4;
                    int part_pos = h_real_to_part[real_idx];
                    ofs << "[b" << body_idx << ":n" << sub_node
                        << " r=" << real_idx << " p=" << part_pos << "] ";
                }
                ofs << "\n";
            }

            // Check: for each body, are all 4 nodes in the same cluster?
            ofs << "\n# Body split check (bodies whose nodes span multiple clusters):\n";
            for(SizeT b = 0; b < body_count; b++)
            {
                int c0 = part_ids[b * 4 + 0];
                bool split = false;
                for(int k = 1; k < 4; k++)
                    if(part_ids[b * 4 + k] != c0) { split = true; any_split = true; }
                if(split)
                    ofs << "  SPLIT body " << b << ": clusters "
                        << part_ids[b*4+0] << " " << part_ids[b*4+1] << " "
                        << part_ids[b*4+2] << " " << part_ids[b*4+3] << "\n";
            }
            if(!any_split)
                ofs << "  (none — all bodies are fully within one cluster)\n";

            logger::info("ABDMASPreconditioner: partition map dumped to {}", dump_path);
        }
    }

    // -------------------------------------------------------------------------
    // do_assemble
    // -------------------------------------------------------------------------
    // Passes the raw ABD BCOO matrix directly into the MAS engine.
    //
    // ABD BCOO layout (upper-triangle only):
    //   Intra-body: 10 triplets per body at (4b+k, 4b+l), k ≤ l.
    //   Inter-body: 16 triplets per joint at (4bi+k, 4bj+l), all k,l, bi < bj.
    //
    // The engine symmetrises on the fly (adds H^T for lower-triangle positions).
    virtual void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) override
    {
        UIPC_ASSERT(m_engine.is_initialized(),
                    "ABDMASPreconditioner: engine not initialized.");

        auto A          = info.A();
        int  dof_offset = static_cast<int>(info.dof_offset()) / 3;
        int  trip_count = static_cast<int>(A.triplet_count());

        fill_identity_indices(m_sorted_indices, trip_count);
        m_engine.set_preconditioner(A.values(),
                                    A.row_indices(),
                                    A.col_indices(),
                                    m_sorted_indices.view(),
                                    dof_offset,
                                    0);

        auto dump_mas =
            world().scene().config().find<IndexT>("extras/debug/dump_mas_matrices");
        if(dump_mas && dump_mas->view()[0] != 0)
        {
            auto& cuda_engine =
                static_cast<SimEngine&>(uipc::backend::SimSystem::engine());
            backend::BackendPathTool path_tool{std::string{workspace()}};
            auto out_dir =
                path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "debug");
            if(cuda_engine.frame() == 1 && cuda_engine.newton_iter() == 0)
            {
                m_engine.dump_cluster_matrices_debug(
                    out_dir, cuda_engine.frame(), cuda_engine.newton_iter());

                std::filesystem::create_directories(out_dir);
                std::vector<Eigen::Matrix3d> h_vals(trip_count);
                std::vector<int>             h_rows(trip_count);
                std::vector<int>             h_cols(trip_count);
                A.values().subview(0, trip_count).copy_to(h_vals.data());
                A.row_indices().subview(0, trip_count).copy_to(h_rows.data());
                A.col_indices().subview(0, trip_count).copy_to(h_cols.data());

                auto bcoo_path = (out_dir / "abd_bcoo.f1.n0.txt").string();
                std::ofstream o(bcoo_path);
                o << "# ABD fine BCOO upper-triangle, dof_offset=" << dof_offset << "\n";
                o << "# trip_count=" << trip_count << "\n";
                o << "# Format: row col h00 h01 h02 h10 h11 h12 h20 h21 h22\n";
                for(int i = 0; i < trip_count; i++)
                {
                    int r = h_rows[i] - dof_offset;
                    int c = h_cols[i] - dof_offset;
                    o << r << " " << c;
                    for(int ii = 0; ii < 3; ii++)
                        for(int jj = 0; jj < 3; jj++)
                            o << " " << h_vals[i](ii, jj);
                    o << "\n";
                }
                logger::info("ABDMASPreconditioner: BCOO dumped to {}", bcoo_path);
            }
        }
    }

    // -------------------------------------------------------------------------
    // do_apply
    // -------------------------------------------------------------------------
    // z = M^{-1} r — multi-level Schwarz solve on raw ABD DOF vectors.
    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override
    {
        UIPC_ASSERT(m_engine.is_initialized(),
                    "ABDMASPreconditioner: engine not initialized.");

        m_engine.apply(info.r(), info.z(), info.converged());
    }
};

}  // namespace uipc::backend::cuda

// SimSystemCreator must come after the full class definition so the
// std::derived_from<ISimSystem> constraint is satisfiable.
namespace uipc::backend
{
template <>
class SimSystemCreator<cuda::ABDMASPreconditioner>
{
  public:
    static U<cuda::ABDMASPreconditioner> create(SimEngine& engine)
    {
        auto  scene      = dynamic_cast<cuda::SimEngine&>(engine).world().scene();
        auto  force_diag = scene.config().find<IndexT>("extras/precond/force_abd_diag");
        if(force_diag && force_diag->view()[0] != 0)
            return nullptr;
        return uipc::make_unique<cuda::ABDMASPreconditioner>(engine);
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDMASPreconditioner);
}  // namespace uipc::backend::cuda
