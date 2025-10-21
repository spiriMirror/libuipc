#pragma once
#include <uipc/usd/stage.h>
#include <pxr/usd/usd/stage.h>
#include <private/prim.h>

namespace uipc::usd
{
class Stage::Impl
{
  public:
    explicit Impl(pxr::UsdStageRefPtr usdStage)
        : m_usdStage(std::move(usdStage))
    {
    }
    static S<Stage::Impl> open(std::string_view path);
    pxr::UsdStageRefPtr   m_usdStage;
};
}  // namespace uipc::usd
