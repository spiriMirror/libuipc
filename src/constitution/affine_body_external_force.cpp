#include <uipc/constitution/affine_body_external_force.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/attribute_collection.h>
#include <uipc/common/set.h>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 666;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodyExternalForce",
                           .type = string{builtin::Constraint}});
    return uids;
}


AffineBodyExternalBodyForce::AffineBodyExternalBodyForce(const Json& config)
{
    m_config = config;
}

AffineBodyExternalBodyForce::~AffineBodyExternalBodyForce() = default;

void AffineBodyExternalBodyForce::apply_to(geometry::SimplicialComplex& sc,
                                           const Vector12&              force,
                                           bool                         enable)
{
    // Add to constraint_uids (new constraint-based architecture)
    auto uids = sc.meta().find<VectorXu64>(builtin::constraint_uids);
    if(!uids)
        uids = sc.meta().create<VectorXu64>(builtin::constraint_uids);

    // Add uid to the list
    auto&    vs = geometry::view(*uids).front();
    set<U64> uids_set(vs.begin(), vs.end());
    uids_set.insert(uid());
    vs.resize(uids_set.size());
    std::ranges::copy(uids_set, vs.begin());

    // External forces have a separate switch from soft transform constraints.
    auto force_enabled = sc.instances().find<IndexT>("external_force/is_constrained");
    if(!force_enabled)
    {
        force_enabled = sc.instances().create<IndexT>("external_force/is_constrained",
                                                      static_cast<IndexT>(enable));
    }
    std::ranges::fill(view(*force_enabled), static_cast<IndexT>(enable));

    // Create external_force attribute
    auto external_force_attr = sc.instances().find<Vector12>("external_force");
    if(!external_force_attr)
    {
        external_force_attr =
            sc.instances().create<Vector12>("external_force", Vector12::Zero());
    }

    auto external_force_view = view(*external_force_attr);
    std::ranges::fill(external_force_view, force);
}

void AffineBodyExternalBodyForce::apply_to(geometry::SimplicialComplex& sc,
                                           const Vector3&               force,
                                           bool                         enable)
{
    Vector12 force12      = Vector12::Zero();
    force12.segment<3>(0) = force;  // Only translational force
    apply_to(sc, force12, enable);
}

Json AffineBodyExternalBodyForce::default_config()
{
    return Json::object();
}

U64 AffineBodyExternalBodyForce::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution
