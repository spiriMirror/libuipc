#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/io/simplicial_complex_io.h>
#include <uipc/geometry.h>
#include <uipc/geometry/geometry_atlas.h>
#include <uipc/common/format.h>
#include <fstream>
#include <uipc/builtin/attribute_name.h>

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

TEST_CASE("geometry_atlas_commit", "[serialization]")
{
    Json json;

    SimplicialComplexIO io;
    auto mesh = io.read_obj(fmt::format("{}cube.obj", AssetDir::trimesh_path()));

    SimplicialComplex mesh_copy = mesh;
    mesh_copy.instances().resize(2);

    GeometryCommit gc = mesh_copy - mesh;

    AttributeCollection ac;
    ac.resize(10);
    auto my_data = ac.create<IndexT>("I", 0);

    AttributeCollection ac_copy = ac;
    ac_copy.create<IndexT>("II", 0);
    ac_copy.resize(20);

    AttributeCollectionCommit ac_commit = ac_copy - ac;

    GeometryAtlasCommit gac;

    gac.create(gc);
    gac.create("ac", ac_commit);

    json = gac.to_json();


    // Write the json to a file
    {
        auto output_path = AssetDir::output_path(__FILE__);
        auto output_file = fmt::format("{}/geometry_atlas_commit.json", output_path);

        std::ofstream ofs(output_file);
        ofs << json.dump(4);
    }

    {
        GeometryAtlasCommit gac2;
        gac2.from_json(json);

        auto ac2 = gac2.find("ac");
        REQUIRE(ac2 != nullptr);

        const AttributeCollection& inc2  = ac2->attribute_collection();
        auto                       names = inc2.names();
        REQUIRE(names.size() == 2);

        std::sort(names.begin(), names.end());

        REQUIRE(std::find(names.begin(), names.end(), "I") != names.end());
        REQUIRE(std::find(names.begin(), names.end(), "II") != names.end());

        auto gc2 = gac2.find(0);
        REQUIRE(gc2 != nullptr);

        REQUIRE(gc2->is_modification());
        REQUIRE(gc2->is_valid());
        REQUIRE(gc2->new_geometry() == nullptr);

        auto instances2_it = gc2->attribute_collections().find("instances");
        REQUIRE(instances2_it != gc2->attribute_collections().end());

        REQUIRE(instances2_it->second->attribute_collection().size() == 2);
    }
}
