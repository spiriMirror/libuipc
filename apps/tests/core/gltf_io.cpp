#include <app/asset_dir.h>
#include <app/test_common.h>
#include <tiny_gltf.h>
#include <uipc/io/gltf_io.h>

using namespace uipc;

TEST_CASE("gltf_io", "[util]")
{
    auto path = AssetDir::output_path(__FILE__);
    REQUIRE(test_gltf(path) == 0);
}