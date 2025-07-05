#include <contact_system/contact_exporter.h>
#include <contact_system/contact_exporter_manager.h>

namespace uipc::backend::cuda
{
std::string_view ContactExporter::prim_type() const noexcept
{
    return get_prim_type();
}

void ContactExporter::do_build()
{
    auto& manager = require<ContactExporterManager>();

    BuildInfo info;
    do_build(info);

    manager.add_exporter(this);
}

void ContactExporter::contact_energy(std::string_view prim_type, geometry::Geometry& energy)
{
    get_contact_energy(prim_type, energy);
}

void ContactExporter::contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad)
{
    get_contact_gradient(prim_type, vert_grad);
}

void ContactExporter::contact_hessian(std::string_view prim_type, geometry::Geometry& vert_hess)
{
    get_contact_hessian(prim_type, vert_hess);
}
}  // namespace uipc::backend::cuda
