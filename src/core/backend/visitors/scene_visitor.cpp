#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/core/scene.h>
#include <uipc/core/internal/scene.h>

namespace uipc::backend
{
SceneVisitor::SceneVisitor(core::Scene& scene) noexcept
    : SceneVisitor(*scene.m_internal)
{
}

SceneVisitor::SceneVisitor(core::internal::Scene& scene) noexcept
    : m_scene(scene)
    , m_diff_sim_visitor(scene.diff_sim())
{
    m_ref = S<core::Scene>{new core::Scene(scene.shared_from_this())};
}

void SceneVisitor::begin_pending() noexcept
{
    m_scene.begin_pending();
}

void SceneVisitor::solve_pending() noexcept
{
    m_scene.solve_pending();
}

bool SceneVisitor::is_pending() const noexcept
{
    return m_scene.is_pending();
}

span<S<geometry::GeometrySlot>> SceneVisitor::geometries() const noexcept
{
    return m_scene.geometries().geometry_slots();
}

span<S<geometry::GeometrySlot>> SceneVisitor::pending_geometries() const noexcept
{
    return m_scene.geometries().pending_create_slots();
}

S<geometry::GeometrySlot> SceneVisitor::find_geometry(IndexT id) noexcept
{
    return m_scene.geometries().find(id);
}

S<geometry::GeometrySlot> SceneVisitor::find_rest_geometry(IndexT id) noexcept
{
    return m_scene.rest_geometries().find(id);
}

span<S<geometry::GeometrySlot>> SceneVisitor::rest_geometries() const noexcept
{
    return m_scene.rest_geometries().geometry_slots();
}

span<S<geometry::GeometrySlot>> SceneVisitor::pending_rest_geometries() const noexcept
{
    return m_scene.rest_geometries().pending_create_slots();
}

span<IndexT> SceneVisitor::pending_destroy_ids() const noexcept
{
    return m_scene.geometries().pending_destroy_ids();
}

const Json& SceneVisitor::info() const noexcept
{
    return m_scene.config();
}

const core::ConstitutionTabular& SceneVisitor::constitution_tabular() const noexcept
{
    return m_scene.constitution_tabular();
}

core::ConstitutionTabular& SceneVisitor::constitution_tabular() noexcept
{
    return m_scene.constitution_tabular();
}

const core::ContactTabular& SceneVisitor::contact_tabular() const noexcept
{
    return m_scene.contact_tabular();
}

core::ContactTabular& SceneVisitor::contact_tabular() noexcept
{
    return m_scene.contact_tabular();
}

const DiffSimVisitor& SceneVisitor::diff_sim() const noexcept
{
    return m_diff_sim_visitor;
}

DiffSimVisitor& SceneVisitor::diff_sim() noexcept
{
    return m_diff_sim_visitor;
}

core::Scene& uipc::backend::SceneVisitor::ref() noexcept
{
    return *m_ref;
}
}  // namespace uipc::backend
