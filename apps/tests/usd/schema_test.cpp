#include <app/test_common.h>
#include <app/asset_dir.h>
#include <fmt/color.h>
#include <fmt/core.h>
#include <uipc/uipc.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <uipc/usd/schema.h>

using namespace uipc;

TEST_CASE("affine_body_api", "[usd]")
{
    auto stage = pxr::UsdStage::CreateNew("Hello.usda");
    auto prim  = pxr::UsdGeomXform::Define(stage, pxr::SdfPath("/Cube"));
    auto api = usd::AffineBodyAPI::Apply(prim.GetPrim());
    auto attr = api.GetConstitution_uidAttr();
    REQUIRE(attr.IsValid());
    U64 uid = 0;
    attr.Get(&uid);
    REQUIRE(uid == 1);
}
