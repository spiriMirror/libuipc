#include <uipc/core/contact_model.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/io/simplicial_complex_io.h>
#include <sanity_checker.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/io/spread_sheet_io.h>
#include <context.h>
#include <uipc/geometry/utils/bvh.h>
#include <uipc/geometry/utils/intersection.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/map.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/constitution_uid_collection.h>

namespace std
{
// Vector2i  set comparison
template <>
struct less<uipc::Vector2i>
{
    bool operator()(const uipc::Vector2i& lhs, const uipc::Vector2i& rhs) const
    {
        return lhs[0] < rhs[0] || (lhs[0] == rhs[0] && lhs[1] < rhs[1]);
    }
};
}  // namespace std

namespace uipc::sanity_check
{
class SimplicialVolumeCheck final : public SanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 4;
    using SanityChecker::SanityChecker;

  protected:
    virtual void build(backend::SceneVisitor& scene) override {}

    virtual U64 get_id() const noexcept override { return SanityCheckerUID; }

    virtual SanityCheckResult do_check(backend::SceneVisitor& scene,
                                       backend::SanityCheckMessageVisitor& msg) noexcept override
    {
        auto context = find<Context>();

        auto geo_slots = scene.geometries();

        vector<IndexT> invalid_geo_ids;
        vector<IndexT> invalid_obj_ids;

        for(auto& geo_slot : geo_slots)
        {
            auto& geo = geo_slot->geometry();
            if(geo.type() != builtin::SimplicialComplex)
                continue;

            auto sc = geo.as<geometry::SimplicialComplex>();
            UIPC_ASSERT(sc, "Cannot cast to simplicial complex, why can this happen?");

            auto cuid = sc->meta().find<U64>(builtin::constitution_uid);
            if(!cuid)
                continue;

            auto& uid_info =
                builtin::ConstitutionUIDCollection::instance().find(cuid->view()[0]);
            auto gid = geo_slot->id();
            auto oid = sc->meta().find<IndexT>("sanity_check/object_id")->view()[0];

            if(uid_info.type == builtin::FiniteElement)
            {
                auto volume      = sc->vertices().find<Float>(builtin::volume);
                auto volume_view = volume->view();
                auto min_elem =
                    std::min_element(volume_view.begin(), volume_view.end());

                if(*min_elem <= 0.0)
                {
                    invalid_geo_ids.push_back(gid);
                    invalid_obj_ids.push_back(oid);
                }
            }
            else if(uid_info.type == builtin::AffineBody)
            {

                auto volume      = sc->instances().find<Float>(builtin::volume);
                auto volume_view = volume->view();
                auto min_elem =
                    std::min_element(volume_view.begin(), volume_view.end());
                if(*min_elem <= 0.0)
                {
                    invalid_geo_ids.push_back(gid);
                    invalid_obj_ids.push_back(oid);
                }
            }
        }

        if(invalid_geo_ids.empty())
            return SanityCheckResult::Success;

        auto& buffer = msg.message();
        for(IndexT i = 0; i < invalid_geo_ids.size(); ++i)
        {
            auto obj_name = objects().find(invalid_obj_ids[i])->name();
            fmt::format_to(std::back_inserter(buffer),
                           "Geometry({}) in Object[{}({})] has non-positive volume.\n",
                           invalid_geo_ids[i],
                           obj_name,
                           invalid_obj_ids[i]);
        }
        fmt::format_to(std::back_inserter(buffer),
                       "Copy valid geometry with name `ObjectID.GeometryID` for post-processing.");

        for(IndexT i = 0; i < invalid_geo_ids.size(); ++i)
        {
            auto geo_slot = scene.find_geometry(invalid_geo_ids[i]);
            auto sc = geo_slot->geometry().as<geometry::SimplicialComplex>();

            msg.geometries()[fmt::format("{}.{}", invalid_obj_ids[i], invalid_geo_ids[i])] =
                std::make_shared<geometry::SimplicialComplex>(*sc);
        }

        return SanityCheckResult::Error;
    };
};

REGISTER_SANITY_CHECKER(SimplicialVolumeCheck);
}  // namespace uipc::sanity_check
