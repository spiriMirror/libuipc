#pragma once
#include <uipc/backend/rmr/manager.h>
#include <uipc/geometry/geometry.h>
#include <uipc/backend/visitors/scene_visitor.h>

namespace uipc::backend
{
class GeometryCollector : public IManager
{
  public:
    GeometryCollector(SceneVisitor visitor) noexcept;

  private:
    SceneVisitor m_visitor;
};
}  // namespace uipc::backend
