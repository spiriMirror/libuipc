#include <app/app.h>
#include <uipc/usd/stage.h>
#include <uipc/common/exception.h>
#include <fstream>
using namespace uipc;

TEST_CASE("stage_prim", "[usd]")
{
    auto output = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);
    auto path   = fmt::format("{}/HelloStage.usda", output);
    std::ofstream(path) << "#usda 1.0\ndef Xform \"Cube\" {}\n";
    auto stage = usd::Stage::open(path);
    REQUIRE_NOTHROW(stage.get_prim_at_path("/Cube"));
    REQUIRE_THROWS_AS(usd::Stage::open(path + ".missing"), Exception);
}
