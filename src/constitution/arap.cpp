#include <uipc/constitution/arap.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/constitution/conversion.h>
#include <uipc/common/log.h>
#include <uipc/builtin/constitution_type.h>

namespace uipc::constitution
{
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid = 9, .name = "ARAP", .type = string{builtin::FiniteElement}});
    return uids;
}

ARAP::ARAP(const Json& config) noexcept
    : m_config(config)
{
}

void ARAP::apply_to(geometry::SimplicialComplex& sc, Float kappa, Float mass_density) const
{
    Base::apply_to(sc, mass_density);

    UIPC_ASSERT(sc.dim() == 3, "Now ARAP only supports 3D simplicial complex");

    auto kappa_attr = sc.tetrahedra().find<Float>("kappa");
    if(!kappa_attr)
        kappa_attr = sc.tetrahedra().create<Float>("kappa", kappa);
    std::ranges::fill(geometry::view(*kappa_attr), kappa);
}

Json ARAP::default_config() noexcept
{
    return Json::object();
}

U64 ARAP::get_uid() const noexcept
{
    return 9;
}
}  // namespace uipc::constitution
