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
    bool                    m_has_partition = false;
    muda::DeviceBuffer<uint32_t> sorted_indices;

    virtual void do_build(BuildInfo& info) override
    {
        finite_element_method       = &require<FiniteElementMethod>();
        global_linear_system        = &require<GlobalLinearSystem>();
        fem_linear_subsystem        = &require<FEMLinearSubsystem>();
        auto& global_vertex_manager = require<GlobalVertexManager>();

        // Check if any geometry has the mesh_part attribute
        auto geo_slots      = world().scene().geometries();
        bool found_partition = false;
        for(SizeT i = 0; i < geo_slots.size(); i++)
        {
            auto& geo = geo_slots[i]->geometry();
            auto* sc  = geo.as<geometry::SimplicialComplex>();
            if(sc)
            {
                auto mesh_part = sc->vertices().find<IndexT>("mesh_part");
                if(mesh_part)
                {
                    found_partition = true;
                    break;
                }
            }
        }

        if(!found_partition)
        {
            throw SimSystemException(
                "FEMMASPreconditioner: No 'mesh_part' attribute found. "
                "Call mesh_partition() before world.init().");
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
            for(SizeT v = 0; v < geo_info.vertex_count; v++)
                part_ids[geo_info.vertex_offset + v] = part_view[v];
        }

        if(!has_parts)
        {
            m_has_partition = false;
            return;
        }

        // ---- 4. Build partition-ordered index mappings ----

        IndexT max_part_id = 0;
        for(auto pid : part_ids)
            if(pid > max_part_id) max_part_id = pid;

        std::vector<std::vector<int>> part_blocks(max_part_id + 1);
        for(SizeT i = 0; i < vert_num; i++)
            if(part_ids[i] >= 0)
                part_blocks[part_ids[i]].push_back(static_cast<int>(i));

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

        auto A          = info.A();
        int  dof_offset = static_cast<int>(info.dof_offset());

        auto triplet_count = A.triplet_count();
        auto values_view   = A.values();
        auto row_view      = A.row_indices();
        auto col_view      = A.col_indices();

        auto* values  = reinterpret_cast<const Eigen::Matrix3d*>(values_view.data());
        auto* row_ids = reinterpret_cast<const int*>(row_view.data());
        auto* col_ids = reinterpret_cast<const int*>(col_view.data());

        fill_identity_indices(sorted_indices, triplet_count);

        engine.set_preconditioner(values,
                                  row_ids,
                                  col_ids,
                                  reinterpret_cast<const uint32_t*>(sorted_indices.data()),
                                  dof_offset / 3,
                                  static_cast<int>(triplet_count),
                                  0);
    }

    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override
    {
        if(!m_has_partition || !engine.is_initialized())
            return;

        engine.apply(info.r(), info.z());
    }
};

REGISTER_SIM_SYSTEM(FEMMASPreconditioner);
}  // namespace uipc::backend::cuda
