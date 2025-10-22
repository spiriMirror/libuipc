#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/usd/stage.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
using namespace uipc;

auto build_stage()
{
    using namespace pxr;
    auto output_path = AssetDir::output_path(__FILE__);
    auto usd_path    = fmt::format("{}/HelloStage.usda", output_path);
    auto usd_stage   = UsdStage::CreateNew(usd_path);
    auto cube        = UsdGeomXform::Define(usd_stage, SdfPath("/Cube"));
    usd_stage->Save();
    return usd_path;
}

TEST_CASE("stage_prim", "[usd]")
{
    auto usd_path = build_stage();
    auto stage    = usd::Stage::open(usd_path);
}
