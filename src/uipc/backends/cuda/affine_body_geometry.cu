#include <affine_body_geometry.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyGeometry);

void AffineBodyGeometry::build()
{
    // find dependent systems
    m_global_vertex_manager = find<GlobalVertexManager>();

    // Register the action to initialize the affine body geometry
    on_init_scene([this] { init_affine_body_geometry(); });

    // Register the action to build the vertex info
    m_global_vertex_manager->on_update([this](VertexCountInfo& info)
                                       { report_vertex_count(info); },
                                       [this](const GlobalVertexInfo& info)
                                       { receive_global_vertex_info(info); });
}

void AffineBodyGeometry::init_affine_body_geometry()
{
    auto& world = this->world();
    auto  scene = world.scene();
    spdlog::info("Initializing Affine Body Geometry");

    // find the affine bodies
    auto geos      = scene.geometries();
    auto rest_geos = scene.rest_geometries();

    
    
}

void AffineBodyGeometry::report_vertex_count(VertexCountInfo& vertex_count_info)
{
}

void AffineBodyGeometry::receive_global_vertex_info(const GlobalVertexInfo& global_vertex_info)
{
}
}  // namespace uipc::backend::cuda