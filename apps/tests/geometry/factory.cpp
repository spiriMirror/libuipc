#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

using namespace uipc;
using namespace uipc::geometry;


TEST_CASE("trimesh", "[factory]")
{
    // vertices
    vector<Vector3> Vs = {
        {-1.0, 0.0, 1.0},  //
        {1.0, 0.0, 1.0},   //
        {1.0, 0.0, -1.0},  //
        {-1.0, 0.0, -1.0}  //
    };

    // quad faces
    vector<Vector4i> Fs = {
        {0, 1, 2, 3},
    };

    auto sc = trimesh(Vs, Fs);

    SimplicialComplexIO io;
    auto                output_path = AssetDir::output_path(__FILE__);
    io.write_obj(fmt::format("{}quad.obj", output_path), sc);
}