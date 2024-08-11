#include <uipc/constitutions/arap.h>
#include <uipc/geometry/utils/compute_vertex_mass.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/constitutions/conversion.h>
#include <uipc/common/log.h>

namespace uipc::constitution
{
ARAP::ARAP(const Json& config) noexcept {}

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

std::string_view ARAP::get_name() const noexcept
{
    return builtin::ConstitutionUIDRegister::instance().find(get_uid()).name;
}
}  // namespace uipc::constitution
