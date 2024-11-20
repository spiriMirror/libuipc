#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

using namespace uipc;
using namespace uipc::geometry;


TEST_CASE("spreadsheet_simple", "[io]")
{
    SimplicialComplexIO io;

    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    label_surface(mesh);
    label_triangle_orient(mesh);

    SpreadSheetIO sio{AssetDir::output_path(__FILE__)};
    // dump to csv
    sio.write_csv("spreadsheet", mesh);
    // dump to json
    sio.write_json("spreadsheet", mesh);
}