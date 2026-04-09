#include <uipc/constitution/finite_element_external_force.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/attribute_collection.h>
#include <uipc/common/set.h>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 671;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "FiniteElementExternalForce",
                           .type = string{builtin::Constraint}});
    return uids;
}

FiniteElementExternalForce::FiniteElementExternalForce(const Json& config)
{
    m_config = config;
}

FiniteElementExternalForce::~FiniteElementExternalForce() = default;

void FiniteElementExternalForce::apply_to(geometry::SimplicialComplex& sc,
                                          const Vector3&               force)
{
    auto uids = sc.meta().find<VectorXu64>(builtin::constraint_uids);
    if(!uids)
        uids = sc.meta().create<VectorXu64>(builtin::constraint_uids);

    auto&    vs = geometry::view(*uids).front();
    set<U64> uids_set(vs.begin(), vs.end());
    uids_set.insert(uid());
    vs.resize(uids_set.size());
    std::ranges::copy(uids_set, vs.begin());

    auto is_constrained_attr = sc.vertices().find<IndexT>(builtin::is_constrained);
    if(!is_constrained_attr)
    {
        is_constrained_attr = sc.vertices().create<IndexT>(builtin::is_constrained, 0);
    }

    auto external_force_attr = sc.vertices().find<Vector3>("external_force");
    if(!external_force_attr)
    {
        external_force_attr =
            sc.vertices().create<Vector3>("external_force", Vector3::Zero());
    }

    auto external_force_view = view(*external_force_attr);
    std::ranges::fill(external_force_view, force);
}

Json FiniteElementExternalForce::default_config()
{
    return Json::object();
}

U64 FiniteElementExternalForce::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution
