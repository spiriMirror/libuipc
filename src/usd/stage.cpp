#include <uipc/usd/stage.h>
#include <private/stage.h>

namespace uipc::usd
{
S<Stage::Impl> Stage::Impl::open(std::string_view path)
{
    auto usdStage = pxr::UsdStage::Open(std::string(path));
    if(!usdStage)
    {
        return nullptr;
    }
    return uipc::make_shared<Stage::Impl>(std::move(usdStage));
}

Stage Stage::open(std::string_view path)
{
    return Stage{Impl::open(path)};
}

Prim Stage::get_prim_at_path(std::string_view path) const
{
    auto usdPrim = m_impl->m_usdStage->GetPrimAtPath(pxr::SdfPath(std::string(path)));
    return Prim{uipc::make_shared<Prim::Impl>(std::move(usdPrim))};
}

Stage::~Stage() = default;

Stage::Stage(S<Impl> impl)
    : m_impl(std::move(impl))
{
}
}  // namespace uipc::usd
