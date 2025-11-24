#include <uipc/constitution/affine_body_external_wrench.h>
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
                           .name = "AffineBodyExternalWrench",
                           .type = string{builtin::ExtraConstitution}});
    return uids;
}

AffineBodyExternalWrench::AffineBodyExternalWrench(const Json& config)
{
    m_config = config;
}

AffineBodyExternalWrench::~AffineBodyExternalWrench() = default;

void AffineBodyExternalWrench::apply_to(geometry::SimplicialComplex& sc, const Vector12& wrench)
{
    // Add to extra_constitution_uids (similar to FiniteElementExtraConstitution)
    auto uids = sc.meta().find<VectorXu64>(builtin::extra_constitution_uids);
    if(!uids)
        uids = sc.meta().create<VectorXu64>(builtin::extra_constitution_uids);

    // Add uid to the list
    auto&    vs = geometry::view(*uids).front();
    set<U64> uids_set(vs.begin(), vs.end());
    uids_set.insert(uid());
    vs.resize(uids_set.size());
    std::ranges::copy(uids_set, vs.begin());

    // Create external wrench attribute
    auto external_wrench = sc.instances().find<Vector12>("external_wrench");
    if(!external_wrench)
    {
        external_wrench = sc.instances().create<Vector12>("external_wrench", Vector12::Zero());
    }

    auto external_wrench_view = view(*external_wrench);
    std::ranges::fill(external_wrench_view, wrench);
}

void AffineBodyExternalWrench::apply_to(geometry::SimplicialComplex& sc, const Vector3& force)
{
    Vector12 wrench = Vector12::Zero();
    wrench.segment<3>(0) = force;  // Only translational force
    apply_to(sc, wrench);
}

Json AffineBodyExternalWrench::default_config()
{
    return Json::object();
}

U64 AffineBodyExternalWrench::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution
