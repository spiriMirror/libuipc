#include <type_define.h>
#include <Eigen/Dense>
#include <linear_system/local_preconditioner.h>
#include <finite_element/finite_element_method.h>
#include <linear_system/global_linear_system.h>
#include <finite_element/fem_linear_subsystem.h>
#include <global_geometry/global_vertex_manager.h>
#include <finite_element/mas_preconditioner_engine.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/log.h>
#include <set>

namespace uipc::backend::cuda
{
// Free function: NVCC on Windows forbids __device__ lambdas in static / internal-linkage functions
void fill_identity_indices(muda::DeviceBuffer<uint32_t>& buf, int count)
{
    using namespace muda;
    buf.resize(count);
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(count,
               [indices = buf.viewer().name("indices")] __device__(int i) mutable
               { indices(i) = static_cast<uint32_t>(i); });
}

void assemble_diag_inv_for_unpartitioned(
    muda::DeviceBuffer<Matrix3x3>&    diag_inv,
    const muda::DeviceBuffer<int>&    unpart_flags,
    muda::CBCOOMatrixView<Float, 3>   A,
    int                               fem_block_offset,
    int                               fem_block_count,
    SizeT                             num_verts)
{
    using namespace muda;

    diag_inv.resize(num_verts);
    diag_inv.fill(Matrix3x3::Identity());

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(A.triplet_count(),
               [triplet    = A.cviewer().name("triplet"),
                diag       = diag_inv.viewer().name("diag_inv"),
                unpart     = unpart_flags.cviewer().name("unpart_flags"),
                fem_offset = fem_block_offset,
                fem_count  = fem_block_count] __device__(int I) mutable
               {
                   auto&& [g_i, g_j, H3x3] = triplet(I);

                   int i = g_i - fem_offset;
                   int j = g_j - fem_offset;

                   if(i < 0 || i >= fem_count || j < 0 || j >= fem_count)
                       return;

                   if(i == j && unpart(i) == 1)
                       diag(i) = eigen::inverse(H3x3);
               });
}

void apply_diag_inv_for_unpartitioned(
    const muda::DeviceBuffer<Matrix3x3>& diag_inv,
    const muda::DeviceBuffer<int>&       unpart_flags,
    muda::DenseVectorView<Float>         z,
    muda::CDenseVectorView<Float>        r,
    SizeT                                num_verts)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(num_verts,
               [r_view = r.viewer().name("r"),
                z_view = z.viewer().name("z"),
                diag   = diag_inv.cviewer().name("diag_inv"),
                unpart = unpart_flags.cviewer().name("unpart_flags")]
               __device__(int i) mutable
               {
                   if(unpart(i) == 1)
                   {
                       z_view.segment<3>(i * 3).as_eigen() =
                           diag(i) * r_view.segment<3>(i * 3).as_eigen();
                   }
               });
}

/**
 * @brief FEM MAS (Multiplicative Additive Schwarz) Preconditioner
 *
 * A multi-level domain-decomposition preconditioner that replaces the
 * simpler diagonal (Jacobi) preconditioner for much better convergence
 * on stiff problems. Based on the StiffGIPC paper.
 *
 * Prerequisites:
 *   The user must call `mesh_partition(sc, 16)` on their SimplicialComplex
 *   BEFORE `world.init()` to create a "mesh_part" vertex attribute.
 *   If this attribute is absent, the system will not be built and the
 *   default FEMDiagPreconditioner will be used instead.
 */
class FEMMASPreconditioner : public LocalPreconditioner
{
  public:
    using LocalPreconditioner::LocalPreconditioner;

    static constexpr int BANKSIZE = MASPreconditionerEngine::BANKSIZE;

  private:
    FiniteElementMethod* finite_element_method = nullptr;
    GlobalLinearSystem*  global_linear_system  = nullptr;
    FEMLinearSubsystem*  fem_linear_subsystem  = nullptr;

    MASPreconditionerEngine engine;
    bool                    m_has_partition       = false;
    bool                    m_has_unpartitioned   = false;
    muda::DeviceBuffer<uint32_t>  sorted_indices;

    // Diagonal fallback for unpartitioned vertices
    muda::DeviceBuffer<Matrix3x3> diag_inv;
    muda::DeviceBuffer<int>       unpartitioned_flags;  // 1 = unpartitioned, 0 = partitioned

    virtual void do_build(BuildInfo& info) override
    {
        finite_element_method       = &require<FiniteElementMethod>();
        global_linear_system        = &require<GlobalLinearSystem>();
        fem_linear_subsystem        = &require<FEMLinearSubsystem>();
        auto& global_vertex_manager = require<GlobalVertexManager>();

        // MAS activates if ANY FEM geometry has mesh_part attribute.
        // Unpartitioned meshes get diagonal (block-Jacobi) fallback internally.
        auto geo_slots       = world().scene().geometries();
        bool found_any_part  = false;
        for(SizeT i = 0; i < geo_slots.size(); i++)
        {
            auto& geo = geo_slots[i]->geometry();
            auto* sc  = geo.as<geometry::SimplicialComplex>();
            if(sc && sc->dim() >= 1)
            {
                auto mesh_part = sc->vertices().find<IndexT>("mesh_part");
                if(mesh_part)
                {
                    found_any_part = true;
                    break;
                }
            }
        }

        if(!found_any_part)
        {
            throw SimSystemException(
                "FEMMASPreconditioner: No 'mesh_part' attribute found on any geometry.");
        }

        info.connect(fem_linear_subsystem);
    }

    virtual void do_init(InitInfo& info) override
    {
        auto& fem = finite_element_method->m_impl;

        SizeT vert_num = fem.xs.size();
        if(vert_num == 0)
            return;

        // ---- 1. Build vertex adjacency from element connectivity ----

        std::vector<std::set<unsigned int>> vert_neighbors(vert_num);

        auto add_edge = [&](IndexT a, IndexT b)
        {
            if(a != b && a >= 0 && b >= 0
               && a < static_cast<IndexT>(vert_num)
               && b < static_cast<IndexT>(vert_num))
            {
                vert_neighbors[a].insert(static_cast<unsigned int>(b));
                vert_neighbors[b].insert(static_cast<unsigned int>(a));
            }
        };

        for(auto& tet : fem.h_tets)
            for(int i = 0; i < 4; i++)
                for(int j = i + 1; j < 4; j++)
                    add_edge(tet[i], tet[j]);

        for(auto& tri : fem.h_codim_2ds)
            for(int i = 0; i < 3; i++)
                for(int j = i + 1; j < 3; j++)
                    add_edge(tri[i], tri[j]);

        for(auto& edge : fem.h_codim_1ds)
            add_edge(edge[0], edge[1]);

        // ---- 2. Build CSR neighbor arrays ----

        std::vector<unsigned int> h_neighbor_list;
        std::vector<unsigned int> h_neighbor_start(vert_num, 0);
        std::vector<unsigned int> h_neighbor_num(vert_num, 0);

        for(SizeT i = 0; i < vert_num; i++)
        {
            h_neighbor_start[i] = static_cast<unsigned int>(h_neighbor_list.size());
            h_neighbor_num[i]   = static_cast<unsigned int>(vert_neighbors[i].size());
            for(auto n : vert_neighbors[i])
                h_neighbor_list.push_back(n);
        }

        // ---- 3. Read mesh_part attribute and build partition mappings ----

        std::vector<IndexT> part_ids(vert_num, -1);
        bool                has_parts = false;

        // Partition IDs are mesh-local (each mesh starts at 0).
        // We add a running offset so that IDs are globally unique
        // across all geometries in the FEM system.
        IndexT partition_offset = 0;

        auto geo_slots = world().scene().geometries();
        for(auto& geo_info : fem.geo_infos)
        {
            auto& geo_slot = geo_slots[geo_info.geo_slot_index];
            auto& geo      = geo_slot->geometry();
            auto* sc       = geo.as<geometry::SimplicialComplex>();
            if(!sc) continue;

            auto mesh_part = sc->vertices().find<IndexT>("mesh_part");
            if(!mesh_part) continue;

            has_parts      = true;
            auto part_view = mesh_part->view();

            IndexT local_max = 0;
            for(SizeT v = 0; v < geo_info.vertex_count; v++)
            {
                IndexT local_pid = part_view[v];
                part_ids[geo_info.vertex_offset + v] = local_pid + partition_offset;
                local_max = std::max(local_max, local_pid);
            }
            partition_offset += local_max + 1;  // next mesh starts after this mesh's max
        }

        if(!has_parts)
        {
            m_has_partition = false;
            return;
        }

        // Check if any vertices are unpartitioned (part_ids[v] == -1)
        {
            std::vector<int> h_unpart_flags(vert_num, 0);
            bool any_unpartitioned = false;
            for(SizeT i = 0; i < vert_num; i++)
            {
                if(part_ids[i] < 0)
                {
                    h_unpart_flags[i] = 1;
                    any_unpartitioned = true;
                }
            }
            m_has_unpartitioned = any_unpartitioned;
            unpartitioned_flags.resize(vert_num);
            unpartitioned_flags.view().copy_from(h_unpart_flags.data());
        }

        // ---- 4. Build partition-ordered index mappings ----

        IndexT max_part_id = 0;
        for(auto pid : part_ids)
            if(pid > max_part_id) max_part_id = pid;

        std::vector<std::vector<int>> part_blocks(max_part_id + 1);
        for(SizeT i = 0; i < vert_num; i++)
            if(part_ids[i] >= 0)
                part_blocks[part_ids[i]].push_back(static_cast<int>(i));

        // Validate: no partition block should exceed BANKSIZE.
        // If this fires, partition IDs from different meshes are colliding.
        for(SizeT b = 0; b < part_blocks.size(); b++)
        {
            UIPC_ASSERT(static_cast<int>(part_blocks[b].size()) <= BANKSIZE,
                         "MAS: partition {} has {} vertices (max {}). "
                         "Partition IDs from different meshes may be colliding — "
                         "need global offset.",
                         b,
                         part_blocks[b].size(),
                         BANKSIZE);
        }

        // Each partition block is padded to BANKSIZE alignment
        int part_map_size = 0;
        for(auto& block : part_blocks)
        {
            int padded = (static_cast<int>(block.size()) + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
            part_map_size += padded;
        }

        std::vector<int> h_part_to_real(part_map_size, -1);
        std::vector<int> h_real_to_part(vert_num, -1);

        int offset = 0;
        for(auto& block : part_blocks)
        {
            for(SizeT i = 0; i < block.size(); i++)
            {
                int real_idx                  = block[i];
                h_part_to_real[offset + (int)i] = real_idx;
                h_real_to_part[real_idx]        = offset + static_cast<int>(i);
            }
            int padded = (static_cast<int>(block.size()) + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
            offset += padded;
        }

        // ---- 5. Initialize the engine ----

        engine.init_neighbor(static_cast<int>(vert_num),
                             static_cast<int>(h_neighbor_list.size()),
                             part_map_size,
                             h_neighbor_list,
                             h_neighbor_start,
                             h_neighbor_num,
                             h_part_to_real,
                             h_real_to_part);

        engine.init_matrix();
        m_has_partition = true;
    }

    virtual void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) override
    {
        if(!m_has_partition || !engine.is_initialized())
            return;

        using namespace muda;

        auto A          = info.A();
        int  dof_offset = static_cast<int>(info.dof_offset());

        auto triplet_count = A.triplet_count();
        auto values_view   = A.values();
        auto row_view      = A.row_indices();
        auto col_view      = A.col_indices();

        auto* values  = reinterpret_cast<const Eigen::Matrix3d*>(values_view.data());
        auto* row_ids = reinterpret_cast<const int*>(row_view.data());
        auto* col_ids = reinterpret_cast<const int*>(col_view.data());

        // MAS assembly for partitioned vertices
        fill_identity_indices(sorted_indices, triplet_count);
        engine.set_preconditioner(values,
                                  row_ids,
                                  col_ids,
                                  reinterpret_cast<const uint32_t*>(sorted_indices.data()),
                                  dof_offset / 3,
                                  static_cast<int>(triplet_count),
                                  0);

        // Diagonal fallback assembly for unpartitioned vertices
        if(m_has_unpartitioned)
        {
            SizeT num_verts      = finite_element_method->xs().size();
            int fem_block_offset = dof_offset / 3;
            int fem_block_count  = static_cast<int>(info.dof_count()) / 3;

            assemble_diag_inv_for_unpartitioned(
                diag_inv, unpartitioned_flags, A,
                fem_block_offset, fem_block_count, num_verts);
        }
    }

    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override
    {
        if(!m_has_partition || !engine.is_initialized())
            return;

        using namespace muda;

        // MAS for partitioned vertices
        engine.apply(info.r(), info.z());

        // Diagonal fallback for unpartitioned vertices
        if(m_has_unpartitioned)
        {
            apply_diag_inv_for_unpartitioned(
                diag_inv, unpartitioned_flags, info.z(), info.r(),
                diag_inv.size());
        }
    }
};

REGISTER_SIM_SYSTEM(FEMMASPreconditioner);
}  // namespace uipc::backend::cuda
