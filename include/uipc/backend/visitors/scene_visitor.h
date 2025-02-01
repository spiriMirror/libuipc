#pragma once
#include <uipc/common/type_define.h>
#include <uipc/core/object.h>
#include <uipc/common/set.h>
#include <uipc/common/unordered_map.h>
#include <uipc/geometry/geometry_collection.h>
#include <uipc/core/constitution_tabular.h>
#include <uipc/core/contact_tabular.h>
#include <uipc/backend/visitors/diff_sim_visitor.h>

namespace uipc::core
{
class Scene;
}

namespace uipc::backend
{
class UIPC_CORE_API SceneVisitor
{
  public:
    SceneVisitor(core::Scene& scene) noexcept;
    void begin_pending() noexcept;
    void solve_pending() noexcept;
    bool is_pending() const noexcept;

    span<S<geometry::GeometrySlot>> geometries() const noexcept;
    S<geometry::GeometrySlot>       find_geometry(IndexT id) noexcept;
    span<S<geometry::GeometrySlot>> pending_geometries() const noexcept;

    span<S<geometry::GeometrySlot>> rest_geometries() const noexcept;
    S<geometry::GeometrySlot>       find_rest_geometry(IndexT id) noexcept;
    span<S<geometry::GeometrySlot>> pending_rest_geometries() const noexcept;

    span<IndexT> pending_destroy_ids() const noexcept;
    const Json&  info() const noexcept;

    const core::ConstitutionTabular& constitution_tabular() const noexcept;
    core::ConstitutionTabular&       constitution_tabular() noexcept;

    const core::ContactTabular& contact_tabular() const noexcept;
    core::ContactTabular&       contact_tabular() noexcept;

    const DiffSimVisitor& diff_sim() const noexcept;
    DiffSimVisitor&       diff_sim() noexcept;

  private:
    core::Scene&   m_scene;
    DiffSimVisitor m_diff_sim;
};
}  // namespace uipc::backend
