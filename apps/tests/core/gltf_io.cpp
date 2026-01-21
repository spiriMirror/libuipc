#include <app/asset_dir.h>
#include <app/app.h>
#include <tiny_gltf.h>
#include <uipc/io/gltf_io.h>

using namespace uipc;

TEST_CASE("gltf_io", "[util]")
{
    auto path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);
    REQUIRE(test_gltf(path) == 0);
}
