#include <uipc/constitution/hookean_spring.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/constitution/conversion.h>
#include <uipc/common/log.h>

namespace uipc::constitution
{
constexpr U64 HookeanSpringUID = 12ull;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = HookeanSpringUID,
                           .name = "HookeanSpring",
                           .type = string{builtin::FiniteElement}});
    return uids;
}

HookeanSpring::HookeanSpring(const Json& config) noexcept
    : m_config(config)
{
}

void HookeanSpring::apply_to(geometry::SimplicialComplex& sc,
                             Float                        kappa,
                             Float                        mass_density,
                             Float                        thickness) const
{
    Base::apply_to(sc, mass_density, thickness);

    UIPC_ASSERT(sc.dim() == 1, "HookeanSpring only supports 1D simplicial complex");

    auto kappa_attr = sc.edges().find<Float>("kappa");
    if(!kappa_attr)
        kappa_attr = sc.edges().create<Float>("kappa", kappa);
    std::ranges::fill(geometry::view(*kappa_attr), kappa);
}

Json HookeanSpring::default_config() noexcept
{
    return Json::object();
}

U64 HookeanSpring::get_uid() const noexcept
{
    return HookeanSpringUID;
}
}  // namespace uipc::constitution
