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

AffineBodyExternalForce::AffineBodyExternalForce(const Json& config)
{
    m_config = config;
}

AffineBodyExternalForce::~AffineBodyExternalForce() = default;

void AffineBodyExternalForce::apply_to(geometry::SimplicialComplex& sc, const Vector12& force)
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

    // Create is_constrained attribute
    auto is_constrained_attr = sc.instances().find<IndexT>(builtin::is_constrained);
    if(!is_constrained_attr)
    {
        is_constrained_attr = sc.instances().create<IndexT>(builtin::is_constrained, 0);
    }

    // Create external_force attribute (renamed from external_wrench)
    auto external_force_attr = sc.instances().find<Vector12>("external_force");
    if(!external_force_attr)
    {
        external_force_attr = sc.instances().create<Vector12>("external_force", Vector12::Zero());
    }

    auto external_force_view = view(*external_force_attr);
    std::ranges::fill(external_force_view, force);
}

void AffineBodyExternalForce::apply_to(geometry::SimplicialComplex& sc, const Vector3& force)
{
    Vector12 force12 = Vector12::Zero();
    force12.segment<3>(0) = force;  // Only translational force
    apply_to(sc, force12);
}

Json AffineBodyExternalForce::default_config()
{
    return Json::object();
}

U64 AffineBodyExternalForce::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution
