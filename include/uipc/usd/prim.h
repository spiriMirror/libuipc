#pragma once
#include <uipc/usd/dllexport.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/geometry.h>

namespace uipc::usd
{
class UIPC_USD_API Prim
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
