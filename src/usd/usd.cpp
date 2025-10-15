#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <uipc/core/scene.h>
#include "schema.h"

namespace uipc::usd
{
void test_usd()
{
    auto stage = pxr::UsdStage::CreateNew("test.usda");
    auto root  = stage->GetRootLayer();
    // add an xform
    auto prim  = pxr::UsdGeomXform::Define(stage, pxr::SdfPath("/Cube"));
    auto scene = uipc::core::Scene();

    auto physicsScene = PhysicsScene::Define(stage, pxr::SdfPath("/PhysicsScene"));
}
}  // namespace uipc::usd