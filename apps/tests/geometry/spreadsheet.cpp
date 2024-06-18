#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/geometry/utils/io.h>
#include <uipc/common/format.h>
#include <uipc/geometry/utils/spreadsheet_io.h>
#include <uipc/geometry/utils/label_surface.h>
#include <uipc/geometry/utils/label_triangle_orient.h>
#include <uipc/geometry/utils/extract_surface.h>
#include <uipc/geometry/utils/flip_inward_triangles.h>

using namespace uipc;
using namespace uipc::geometry;


TEST_CASE("spreadsheet_simple", "[io]")
{
    SimplicialComplexIO io;

    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    //label_surface(mesh);
    //label_triangle_orient(mesh);

    SpreadSheetIO sio{AssetDir::output_path()};
    // dump to csv
    sio.write_csv("spreadsheet_simple", mesh);
    // dump to json
    sio.write_json("spreadsheet_simple", mesh);
}

TEST_CASE("spreadsheet", "[io]")
{
    SimplicialComplexIO io;

    auto mesh =
        io.read_msh(fmt::format("{}cylinder_hole.msh", AssetDir::tetmesh_path()));

    label_surface(mesh);
    label_triangle_orient(mesh);

    // create an index attribute
    auto Is      = mesh.vertices().create<IndexT>("index", 0);
    auto Is_view = view(*Is);
    std::iota(Is_view.begin(), Is_view.end(), 0);

    mesh.meta().create<std::string>("name", "cylinder_hole");

    auto flipped      = flip_inward_triangles(mesh);
    auto mesh_surface = extract_surface(flipped);

    SpreadSheetIO sio{AssetDir::output_path()};

    // dump to csv
    sio.write_csv("spreadsheet", mesh);
    // dump to json
    sio.write_json("spreadsheet", mesh);

    // dump the surface to csv
    sio.write_csv("spreadsheet_surf", mesh_surface);
    // dump the surface to json
    sio.write_json("spreadsheet_surf", mesh_surface);

    // write the surface to obj
    io.write_obj(fmt::format("{}cylinder_hole.obj", AssetDir::output_path()), mesh_surface);
}
