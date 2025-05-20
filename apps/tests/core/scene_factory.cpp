#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/core/scene.h>
#include <uipc/core/scene_factory.h>
#include <uipc/io/simplicial_complex_io.h>
#include <uipc/geometry.h>
#include <uipc/geometry/geometry_atlas.h>
#include <uipc/common/format.h>
#include <fstream>

using namespace uipc;
using namespace uipc::core;
using namespace uipc::geometry;


TEST_CASE("scene_factory", "[serialization]")
{
    SimplicialComplexIO io;
    auto mesh = io.read_obj(fmt::format("{}cube.obj", AssetDir::trimesh_path()));

    auto output_path = AssetDir::output_path(__FILE__);

    Scene scene;

    auto cube = scene.objects().create("cube");
    cube->geometries().create(mesh);

    SceneFactory sf;
    auto         j = sf.to_json(scene);

    {
        auto          output_file = fmt::format("{}/scene.json", output_path);
        std::ofstream ofs(output_file);
        ofs << j.dump(4);
    }

    auto new_scene                 = sf.from_snapshot(sf.from_json(j));
    auto [geo_slot, rest_geo_slot] = new_scene.geometries().find(0);
    auto sc = geo_slot->geometry().as<SimplicialComplex>();

    auto positions        = mesh.positions().view();
    auto loaded_positions = sc->positions().view();

    REQUIRE(positions.size() == loaded_positions.size());
    REQUIRE(vector<Vector3>{positions.begin(), positions.end()}
            == vector<Vector3>{loaded_positions.begin(), loaded_positions.end()});
}