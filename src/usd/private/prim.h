#pragma once
#include <uipc/usd/prim.h>
#include <pxr/usd/usd/prim.h>

namespace uipc::usd
{
class Prim::Impl
{
  public:
    explicit Impl(pxr::UsdPrim usdPrim)
        : m_usdPrim(std::move(usdPrim))
    {
    }
    pxr::UsdPrim m_usdPrim;
};
}  // namespace uipc::usd