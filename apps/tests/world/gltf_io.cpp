#include <app/test_common.h>
#include <tiny_gltf.h>
#include <uipc/util/io/gltf_io.h>

using namespace uipc;

TEST_CASE("gltf_io", "[util]")
{
    REQUIRE(test_gltf() == 0);
}