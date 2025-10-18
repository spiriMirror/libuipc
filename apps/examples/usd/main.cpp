#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <iostream>

int main()
{
    auto stage = pxr::UsdStage::CreateNew("Hello.usda");
    std::cout << stage->GetRootLayer()->GetDisplayName() << std::endl;
    return 0;
}