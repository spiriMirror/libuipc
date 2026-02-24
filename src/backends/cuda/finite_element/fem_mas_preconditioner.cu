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
void fill_identity_indices(muda::DeviceBuffer<uint32_t>& buf, int count)
{
    using namespace muda;
    buf.resize(count);
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(count,
               [indices = buf.viewer().name("indices")] __device__(int i) mutable
               { indices(i) = (uint32_t)i; });
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

    // Host-side data built during init
    bool m_has_partition = false;

    virtual void do_build(BuildInfo& info) override
    {
        finite_element_method       = &require<FiniteElementMethod>();
        global_linear_system        = &require<GlobalLinearSystem>();
        fem_linear_subsystem        = &require<FEMLinearSubsystem>();
        auto& global_vertex_manager = require<GlobalVertexManager>();

        // Check if any geometry has mesh_part attribute
        auto geo_slots = world().scene().geometries();
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
            // No partition data found, this system won't activate
            // The FEMDiagPreconditioner will handle preconditioning instead
            throw SimSystemException("FEMMASPreconditioner: No 'mesh_part' attribute found. "
                                     "Call mesh_partition() before world.init().");
        }

        info.connect(fem_linear_subsystem);
    }

    virtual void do_init(InitInfo& info) override
    {
        auto& fem_impl = finite_element_method->m_impl;

        // Build neighbor list from element connectivity (on host)
        SizeT vertNum = fem_impl.xs.size();

        if(vertNum == 0) return;

        // 1. Collect vertex-to-vertex adjacency from elements
        std::vector<std::set<unsigned int>> vertNeighbors(vertNum);

        auto add_edge = [&](IndexT a, IndexT b)
        {
            if(a != b && a >= 0 && b >= 0
               && a < (IndexT)vertNum && b < (IndexT)vertNum)
            {
                vertNeighbors[a].insert((unsigned int)b);
                vertNeighbors[b].insert((unsigned int)a);
            }
        };

        // From tetrahedra
        for(auto& tet : fem_impl.h_tets)
        {
            for(int i = 0; i < 4; i++)
                for(int j = i + 1; j < 4; j++)
                    add_edge(tet[i], tet[j]);
        }

        // From triangles
        for(auto& tri : fem_impl.h_codim_2ds)
        {
            for(int i = 0; i < 3; i++)
                for(int j = i + 1; j < 3; j++)
                    add_edge(tri[i], tri[j]);
        }

        // From edges
        for(auto& edge : fem_impl.h_codim_1ds)
        {
            add_edge(edge[0], edge[1]);
        }

        // 2. Build CSR neighbor arrays
        std::vector<unsigned int> neighborList;
        std::vector<unsigned int> neighborStart(vertNum, 0);
        std::vector<unsigned int> neighborNum(vertNum, 0);

        for(SizeT i = 0; i < vertNum; i++)
        {
            neighborStart[i] = (unsigned int)neighborList.size();
            neighborNum[i]   = (unsigned int)vertNeighbors[i].size();
            for(auto n : vertNeighbors[i])
                neighborList.push_back(n);
        }
        int totalNeighborNum = (int)neighborList.size();

        // 3. Read mesh_part attribute from scene geometries and build partition mappings
        //    Collect per-vertex partition IDs
        std::vector<IndexT> partIds(vertNum, -1);
        bool has_parts = false;

        auto geo_slots = world().scene().geometries();
        for(auto& geoInfo : fem_impl.geo_infos)
        {
            auto& geo_slot = geo_slots[geoInfo.geo_slot_index];
            auto& geo      = geo_slot->geometry();
            auto* sc       = geo.as<geometry::SimplicialComplex>();
            if(!sc) continue;

            auto mesh_part = sc->vertices().find<IndexT>("mesh_part");
            if(!mesh_part) continue;

            has_parts      = true;
            auto part_view = mesh_part->view();
            for(SizeT v = 0; v < geoInfo.vertex_count; v++)
            {
                partIds[geoInfo.vertex_offset + v] = part_view[v];
            }
        }

        if(!has_parts)
        {
            m_has_partition = false;
            return;
        }

        // 4. Build partId_map_real and real_map_partId
        //    Group vertices by partition, then lay them out in BANKSIZE-aligned blocks

        // Find max partition ID
        IndexT maxPartId = 0;
        for(auto pid : partIds)
            if(pid > maxPartId) maxPartId = pid;

        // Group vertices by partition
        std::vector<std::vector<int>> partBlocks(maxPartId + 1);
        for(SizeT i = 0; i < vertNum; i++)
        {
            if(partIds[i] >= 0)
                partBlocks[partIds[i]].push_back((int)i);
        }

        // Build the partition-ordered mapping
        // Each partition block is padded to BANKSIZE alignment
        int partMapSize = 0;
        for(auto& block : partBlocks)
        {
            int paddedSize = ((int)block.size() + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
            partMapSize += paddedSize;
        }

        std::vector<int> h_partId_map_real(partMapSize, -1);
        std::vector<int> h_real_map_partId(vertNum, -1);

        int offset = 0;
        for(auto& block : partBlocks)
        {
            for(SizeT i = 0; i < block.size(); i++)
            {
                int realIdx = block[i];
                h_partId_map_real[offset + i] = realIdx;
                h_real_map_partId[realIdx]    = offset + (int)i;
            }
            int paddedSize = ((int)block.size() + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
            offset += paddedSize;
        }

        // 5. Initialize the engine
        engine.init_neighbor((int)vertNum,
                             totalNeighborNum,
                             partMapSize,
                             neighborList,
                             neighborStart,
                             neighborNum,
                             h_partId_map_real,
                             h_real_map_partId);

        engine.init_matrix();
        m_has_partition = true;
    }

    virtual void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) override
    {
        if(!m_has_partition || !engine.is_initialized()) return;

        using namespace muda;

        auto A          = info.A();
        int  dof_offset = (int)info.dof_offset();

        // Extract raw pointers from the BCOO matrix views
        auto  triplet_count = A.triplet_count();
        auto  values_view   = A.values();
        auto  row_view      = A.row_indices();
        auto  col_view      = A.col_indices();

        auto* values  = (const Eigen::Matrix3d*)values_view.data();
        auto* row_ids = (const int*)row_view.data();
        auto* col_ids = (const int*)col_view.data();

        // Build identity indices (BCOO is already sorted/unique)
        fill_identity_indices(m_sorted_indices, triplet_count);

        engine.set_preconditioner(values,
                                  row_ids,
                                  col_ids,
                                  (const uint32_t*)m_sorted_indices.data(),
                                  dof_offset / 3,  // block offset
                                  (int)triplet_count,
                                  0);  // cpNum = 0 (no collision pairs for now)
    }

    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override
    {
        if(!m_has_partition || !engine.is_initialized()) return;

        engine.apply(info.r(), info.z());
    }

    muda::DeviceBuffer<uint32_t> m_sorted_indices;
};

REGISTER_SIM_SYSTEM(FEMMASPreconditioner);
}  // namespace uipc::backend::cuda
