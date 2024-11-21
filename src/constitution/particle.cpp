#include <uipc/constitution/particle.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/constitution/conversion.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/common/log.h>

namespace uipc::constitution
{
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid = 13, .name = "Particle", .type = string{builtin::FiniteElement}});
    return uids;
}

Particle::Particle(const Json& config) noexcept
    : m_config(config)
{
}

void Particle::apply_to(geometry::SimplicialComplex& sc, Float mass_density, Float thickness) const
{
    Base::apply_to(sc, mass_density, thickness);

    UIPC_ASSERT(sc.dim() == 0, "Particle only supports 0D simplicial complex");
}

Json Particle::default_config() noexcept
{
    return Json::object();
}

U64 Particle::get_uid() const noexcept
{
    return 13;
}
}  // namespace uipc::constitution
