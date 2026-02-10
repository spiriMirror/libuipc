#include <catch2/catch_all.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/geometry/utils/label_graph_color.h>

using namespace uipc;
using namespace uipc::geometry;

constexpr bool PrintColor = false;

void check_attr(const SimplicialComplex& sc);

void check_no_adjacent_same_color(const SimplicialComplex& sc);

void check_label_graph_color(const SimplicialComplex& sc)
{
    // Check attributes
    check_attr(sc);

    // Check no adjacent same color
    check_no_adjacent_same_color(sc);
}

TEST_CASE("label_graph_color", "[graph_color]")
{
    SECTION("tet")
    {
        std::vector           Vs = {Vector3{0.0, 0.0, 0.0},
                                    Vector3{1.0, 0.0, 0.0},
                                    Vector3{0.0, 1.0, 0.0},
                                    Vector3{0.0, 0.0, 1.0}};
        std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

        auto                  mesh  = tetmesh(Vs, Ts);
        [[maybe_unused]] auto color = label_graph_color(mesh);
        check_label_graph_color(mesh);
    }

    SECTION("tets")
    {
        std::vector           Vs = {Vector3{0.0, 0.0, 0.0},
                                    Vector3{1.0, 0.0, 0.0},
                                    Vector3{0.0, 1.0, 0.0},
                                    Vector3{0.0, 0.0, 1.0}};
        std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

        auto base_mesh = tetmesh(Vs, Ts);
        auto mesh      = merge({&base_mesh, &base_mesh});

        [[maybe_unused]] auto color = label_graph_color(mesh);
        check_label_graph_color(mesh);
    }

    SECTION("cube")
    {
        SimplicialComplexIO io;
        auto mesh = io.read_obj(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
        [[maybe_unused]] auto color = label_graph_color(mesh);
        check_label_graph_color(mesh);
    }
}

void check_attr(const SimplicialComplex& sc)
{
    // According to the specification of label_graph_color, must:

    // 1. have "graph/color" attribute on vertices
    auto graph_color = sc.vertices().find<IndexT>("graph/color");
    REQUIRE(graph_color);

    // 2. have "graph/color_count" attribute on meta
    auto graph_color_count = sc.meta().find<IndexT>("graph/color_count");
    REQUIRE(graph_color_count);

    // 3. color_count == max(color) + 1
    auto       color_view       = graph_color->view();
    const auto color_count_view = graph_color_count->view();
    const auto count = color_view.empty() ? 0 : std::ranges::max(color_view) + 1;
    if constexpr(PrintColor)
    {
        fmt::println("color count: {};", count);
        for(auto&& [i, c] : enumerate(color_view))
        {
            fmt::println("[{}]: {}", i, c);
        }
    }
    REQUIRE(count == color_count_view[0]);
}

void check_no_adjacent_same_color(const SimplicialComplex& sc)
{
    const auto color_attr = sc.vertices().find<IndexT>("graph/color");
    const auto color_view = color_attr->view();
    const auto edges      = sc.edges().topo().view();

    auto success = true;
    for(const auto& edge : edges)
    {
        const auto u = edge[0];
        const auto v = edge[1];
        if(color_view[u] == color_view[v])
        {
            success = false;
            logger::error("Adjacent vertices {} and {} have the same color {}", u, v, color_view[u]);
        }
    }
    REQUIRE(success);
}
