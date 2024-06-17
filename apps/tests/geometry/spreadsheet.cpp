#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/geometry/io.h>
#include <uipc/common/format.h>
#include <uipc/geometry/utils/spreadsheet_io.h>
#include <uipc/geometry/utils/label_surface.h>
#include <uipc/geometry/utils/label_triangle_orient.h>
#include <uipc/geometry/utils/extract_surface.h>
#include <uipc/geometry/utils/flip_inward_triangles.h>

#include <fstream>
using namespace uipc;
using namespace uipc::geometry;
TEST_CASE("spreadsheet", "[io]")
{
    SimplicialComplexIO io;

    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    label_surface(mesh);
    label_triangle_orient(mesh);

    auto flipped = flip_inward_triangles(mesh);
    auto mesh_surface = extract_surface(flipped);

    SpreadSheetIO sio{AssetDir::output_path()};

    // dump to csv
    sio.write_csv("test_spreadsheet", mesh);

    // dump to json
    sio.write_json("test_spreadsheet", mesh);

    sio.write_csv(mesh_surface);
}
