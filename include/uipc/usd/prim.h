#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/geometry.h>

namespace uipc::usd
{
class Prim
{
  public:
    S<geometry::Geometry> to_geometry();
    void                  from_geometry(geometry::Geometry& geo);

  private:
    class Impl;
    friend class Stage;
    explicit Prim(S<Impl> impl);
    S<Impl> m_impl;
};
}  // namespace uipc::usd
