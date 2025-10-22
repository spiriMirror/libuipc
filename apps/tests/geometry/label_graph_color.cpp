#include <catch2/catch_all.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/geometry/utils/label_graph_color.h>

using namespace uipc;
using namespace uipc::geometry;

TEST_CASE("label_graph_color", "[graph_color]")
{
    SECTION("tet")
    {

        std::vector           Vs = {Vector3{0.0, 0.0, 0.0},
                                    Vector3{1.0, 0.0, 0.0},
                                    Vector3{0.0, 1.0, 0.0},
                                    Vector3{0.0, 0.0, 1.0}};
        std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

        auto mesh = tetmesh(Vs, Ts);

        auto color      = label_graph_color(mesh);
        auto color_view = color->view();
        fmt::println("color_view: \n {}", fmt::join(color_view, "\n"));
    }

    SECTION("cube")
    {
        SimplicialComplexIO io;
        auto mesh = io.read_obj(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
        auto color      = label_graph_color(mesh);
        auto color_view = color->view();
        fmt::println("color_view: \n {}", fmt::join(color_view, "\n"));
    }
}
