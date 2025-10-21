#pragma once
#include <uipc/usd/prim.h>

namespace uipc::usd
{
class Stage
{
  public:
    UIPC_USD_API static Stage open(std::string_view path);
    Prim         get_prim_at_path(std::string_view path) const;

  private:
    class Impl;
    explicit Stage(S<Impl> impl);
    S<Impl> m_impl;
};
}  // namespace uipc::usd
