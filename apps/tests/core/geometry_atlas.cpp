#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/io/simplicial_complex_io.h>
#include <uipc/geometry.h>
#include <uipc/geometry/geometry_atlas.h>
#include <uipc/common/format.h>
#include <fstream>

using namespace uipc;
using namespace uipc::geometry;


TEST_CASE("geometry_atlas", "[serialization]")
{
    SimplicialComplexIO io;
    auto mesh = io.read_obj(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    auto mesh_copy = mesh;
    mesh_copy.instances().resize(2);

    AttributeCollection ac;
    auto                my_data = ac.create<IndexT>("I", 0);
    ac.resize(10);

    GeometryAtlas atlas;
    atlas.create("my_data", ac);
    atlas.create(mesh);
    atlas.create(mesh_copy);

    auto output_path = AssetDir::output_path(__FILE__);
    auto output_file = fmt::format("{}/geometry_atlas.json", output_path);


    auto          json = atlas.to_json();
    std::ofstream ofs(output_file);
    ofs << json.dump(4);
    atlas.from_json(json);
    auto json2 = atlas.to_json();

    auto geo = atlas.find(0);
    REQUIRE(geo != nullptr);
    auto sc = geo->geometry().as<SimplicialComplex>();
    REQUIRE(sc != nullptr);
    REQUIRE(sc->to_json() == mesh.to_json());

    auto ac2 = atlas.find("my_data");
    REQUIRE(ac2 != nullptr);
    REQUIRE(ac.to_json() == ac2->to_json());
}
