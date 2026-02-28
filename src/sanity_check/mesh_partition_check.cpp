#include <sanity_checker.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/constitution_uid_collection.h>

namespace uipc::sanity_check
{
class MeshPartitionCheck final : public SanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 5;
    using SanityChecker::SanityChecker;

  protected:
    virtual void build(backend::SceneVisitor& scene) override {}

    virtual U64 get_id() const noexcept override { return SanityCheckerUID; }

    virtual SanityCheckResult do_check(backend::SceneVisitor& scene,
                                       backend::SanityCheckMessageVisitor& msg) noexcept override
    {
        constexpr IndexT MIN_PART_SIZE = 4;
        constexpr IndexT MAX_PART_SIZE = 32;

        auto  geo_slots   = scene.geometries();
        bool  has_error = false;
        auto& buffer      = msg.message();

        for(auto& geo_slot : geo_slots)
        {
            auto& geo = geo_slot->geometry();
            if(geo.type() != builtin::SimplicialComplex)
                continue;

            auto sc = geo.as<geometry::SimplicialComplex>();
            UIPC_ASSERT(sc, "Cannot cast to simplicial complex, why?");

            // Only check FEM constitutions
            auto cuid = sc->meta().find<U64>(builtin::constitution_uid);
            if(!cuid)
                continue;

            auto& uid_info =
                builtin::ConstitutionUIDCollection::instance().find(cuid->view()[0]);
            if(uid_info.type != builtin::FiniteElement)
                continue;

            auto mesh_part = sc->vertices().find<IndexT>("mesh_part");
            if(!mesh_part)
                continue;  // no partition on this FEM geometry, skip

            auto  part_view  = mesh_part->view();
            SizeT vert_count = part_view.size();
            auto  gid        = geo_slot->id();

            if(vert_count == 0)
                continue;

            // Find max partition ID and validate
            IndexT max_part_id = 0;
            for(auto pid : part_view)
            {
                if(pid < 0)
                {
                    fmt::format_to(std::back_inserter(buffer),
                                   "Geometry({}): vertex has negative mesh_part ID {}.\n",
                                   gid, pid);
                    has_error = true;
                }
                max_part_id = std::max(max_part_id, pid);
            }

            if(has_error)
                continue;

            // Count per partition
            vector<SizeT> part_sizes(max_part_id + 1, 0);
            for(auto pid : part_view)
                part_sizes[pid]++;

            // Check for empty partitions (gap in IDs)
            for(IndexT p = 0; p <= max_part_id; ++p)
            {
                if(part_sizes[p] == 0)
                {
                    fmt::format_to(std::back_inserter(buffer),
                                   "Geometry({}): partition {} is empty "
                                   "(IDs are not contiguous).\n",
                                   gid, p);
                    has_error = true;
                }
            }

            SizeT actual_max = *std::max_element(part_sizes.begin(), part_sizes.end());

            // Exceeding max is an error (cluster matrix won't fit in shared memory)
            if(actual_max > MAX_PART_SIZE)
            {
                fmt::format_to(std::back_inserter(buffer),
                               "Geometry({}): max partition size {} exceeds "
                               "the maximum {}.\n",
                               gid, actual_max, MAX_PART_SIZE);
                has_error = true;
            }

            // Below min or not aligned is informational (still works, just suboptimal)
            if(actual_max < MIN_PART_SIZE)
            {
                fmt::format_to(std::back_inserter(buffer),
                               "Geometry({}): max partition size {} is below "
                               "the recommended minimum {} (inefficient clustering).\n",
                               gid, actual_max, MIN_PART_SIZE);
                // info only, not a warning/error
            }

            if(actual_max != 4 && actual_max != 8 && actual_max != 16 && actual_max != 32
               && actual_max >= MIN_PART_SIZE && actual_max <= MAX_PART_SIZE)
            {
                fmt::format_to(std::back_inserter(buffer),
                               "Geometry({}): max partition size {} is not "
                               "aligned to a supported bank size (4, 8, 16, or 32). "
                               "Consider using mesh_partition(sc, 4/8/16/32) for "
                               "optimal preconditioner performance.\n",
                               gid, actual_max);
                // info only, not a warning/error
            }
        }

        if(has_error)
            return SanityCheckResult::Error;
        return SanityCheckResult::Success;
    }
};

REGISTER_SANITY_CHECKER(MeshPartitionCheck);
}  // namespace uipc::sanity_check
