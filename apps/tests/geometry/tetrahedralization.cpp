// Copyright (C) 2026 spiriMirror
// SPDX-License-Identifier: Apache-2.0
#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/geometry/tetrahedralization.h>
#include "../../../src/geometry/tetrahedralization/predicates.h"
#include <array>
#include <map>
#include <set>

namespace
{
using namespace uipc;
using namespace uipc::geometry;
using Grid     = std::array<int, 3>;
using Triangle = std::array<IndexT, 3>;

SimplicialComplex voxel_surface(const std::set<Grid>& occupied)
{
    const int corners[8][3] = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    const int quads[6][4] = {
        {0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
    const int neighbors[6][3] = {
        {0, 0, -1}, {0, 0, 1}, {0, -1, 0}, {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}};
    std::map<Grid, IndexT> ids;
    vector<Vector3>        positions;
    vector<Vector3i>       faces;
    for(auto voxel : occupied)
        for(int face = 0; face < 6; ++face)
        {
            Grid neighbor{voxel[0] + neighbors[face][0],
                          voxel[1] + neighbors[face][1],
                          voxel[2] + neighbors[face][2]};
            if(occupied.contains(neighbor))
                continue;
            IndexT indices[4];
            for(int corner = 0; corner < 4; ++corner)
            {
                auto p = corners[quads[face][corner]];
                Grid point{voxel[0] + p[0], voxel[1] + p[1], voxel[2] + p[2]};
                auto [found, inserted] =
                    ids.emplace(point, static_cast<IndexT>(positions.size()));
                if(inserted)
                    positions.push_back(
                        Vector3{double(point[0]), double(point[1]), double(point[2])});
                indices[corner] = found->second;
            }
            faces.push_back({indices[0], indices[1], indices[2]});
            faces.push_back({indices[0], indices[2], indices[3]});
        }
    return trimesh(positions, faces);
}

void check_volume(const SimplicialComplex& source, const SimplicialComplex& mesh, double volume, bool exact)
{
    auto original  = source.positions().view();
    auto positions = mesh.positions().view();
    REQUIRE(positions.size() >= original.size());
    for(size_t i = 0; i < original.size(); ++i)
        REQUIRE((positions[i].array() == original[i].array()).all());
    auto canonical = [](Triangle f)
    {
        std::sort(f.begin(), f.end());
        return f;
    };
    std::map<Triangle, int> incidence;
    double                  sum = 0;
    for(auto t : mesh.tetrahedra().topo().view())
    {
        double determinant =
            (positions[t[1]] - positions[t[0]])
                .dot((positions[t[2]] - positions[t[0]]).cross(positions[t[3]] - positions[t[0]]));
        REQUIRE(determinant > 0);
        sum += determinant / 6;
        const Triangle fs[4] = {{t[1], t[2], t[3]},
                                {t[0], t[3], t[2]},
                                {t[0], t[1], t[3]},
                                {t[0], t[2], t[1]}};
        for(auto f : fs)
            ++incidence[canonical(f)];
    }
    REQUIRE(sum == Catch::Approx(volume).epsilon(1e-9));
    std::set<Triangle> actual, expected;
    for(auto&& [f, count] : incidence)
    {
        REQUIRE((count == 1 || count == 2));
        if(count == 1)
            actual.insert(f);
    }
    if(exact)
    {
        for(auto f : source.triangles().topo().view())
            expected.insert(canonical({f[0], f[1], f[2]}));
        REQUIRE(actual == expected);
    }
}
}  // namespace

TEST_CASE("native_tetrahedralization_exact_surface", "[tetrahedralization]")
{
    auto surface                 = voxel_surface({{0, 0, 0}});
    auto options                 = tetrahedralization_default_config();
    options["quality_passes"]    = 0;
    options["refinement_budget"] = 0;
    auto [mesh, report]          = tetrahedralize(surface, options);
    check_volume(surface, mesh, 1, true);
    REQUIRE(report["boundary_verified"] == true);
    REQUIRE(report["method"] == "visibility_kernel");
    REQUIRE(mesh.vertices().size() > surface.vertices().size());
}

TEST_CASE("native_tetrahedralization_quality_modes", "[tetrahedralization]")
{
    auto surface = voxel_surface({{0, 0, 0}});
    for(bool preserve : {true, false})
    {
        auto options                  = tetrahedralization_default_config();
        options["preserve_surface"]   = preserve;
        options["target_edge_length"] = 0.4;
        options["refinement_budget"]  = 20;
        auto [mesh, report]           = tetrahedralize(surface, options);
        check_volume(surface, mesh, 1, preserve);
        REQUIRE(report["min_quality"].get<double>() > 0);
        if(!preserve)
            REQUIRE(report["boundary_triangles"].get<size_t>()
                    > surface.triangles().size());
    }
}

TEST_CASE("native_tetrahedralization_nonstar_conservative", "[tetrahedralization]")
{
    // U-shaped volume: its visibility kernel is empty. Surface triangles must
    // still be recovered exactly, without silently replacing the concavity.
    std::set<Grid> occupied{
        {0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {0, 1, 0}, {2, 1, 0}, {0, 2, 0}, {2, 2, 0}};
    auto surface                 = voxel_surface(occupied);
    auto options                 = tetrahedralization_default_config();
    options["quality_passes"]    = 0;
    options["refinement_budget"] = 0;
    auto [mesh, report]          = tetrahedralize(surface, options);
    check_volume(surface, mesh, 7, true);
    REQUIRE(report["method"] == "constrained_advancing_front");
}

TEST_CASE("native_tetrahedralization_cavity", "[tetrahedralization]")
{
    std::set<Grid> occupied;
    for(int x = 0; x < 3; ++x)
        for(int y = 0; y < 3; ++y)
            for(int z = 0; z < 3; ++z)
                if(x != 1 || y != 1 || z != 1)
                    occupied.insert({x, y, z});
    auto surface                 = voxel_surface(occupied);
    auto options                 = tetrahedralization_default_config();
    options["quality_passes"]    = 0;
    options["refinement_budget"] = 0;
    auto [mesh, report]          = tetrahedralize(surface, options);
    check_volume(surface, mesh, 26, true);
}

TEST_CASE("native_tetrahedralization_predicates", "[tetrahedralization]")
{
    using uipc::geometry::tetrahedralization::orient3;
    double  epsilon = std::numeric_limits<double>::epsilon();
    Vector3 a{0, 0, 0}, b{1, 1, 0}, c{1, 1 + 4 * epsilon, 0}, d{0, 0, 1};
    REQUIRE(orient3(a, b, c, d) == 1);
    REQUIRE(orient3(a, c, b, d) == -1);
    REQUIRE(orient3(a, b, c, Vector3{2, 2, 0}) == 0);
    auto surface   = voxel_surface({{0, 0, 0}});
    auto triangles = view(surface.triangles().topo());
    triangles[0]   = triangles[1];
    REQUIRE_THROWS(tetrahedralize(surface));
}

TEST_CASE("native_tetrahedralization_twisted_prism", "[tetrahedralization]")
{
    vector<Vector3>  points;
    constexpr double pi = 3.14159265358979323846;
    for(int layer = 0; layer < 2; ++layer)
        for(int i = 0; i < 3; ++i)
        {
            double angle = 2 * pi * i / 3 + layer * pi / 9;
            points.push_back({std::cos(angle), std::sin(angle), double(layer)});
        }
    vector<Vector3i> fs{{0, 2, 1}, {3, 4, 5}};
    for(int i = 0; i < 3; ++i)
    {
        int next = (i + 1) % 3;
        fs.push_back({i, next, i + 3});
        fs.push_back({next, next + 3, i + 3});
    }
    auto surface        = trimesh(points, fs);
    auto [mesh, report] = tetrahedralize(surface);
    double volume       = compute_mesh_volume(surface);
    check_volume(surface, mesh, volume, true);
    REQUIRE(mesh.vertices().size() > surface.vertices().size());
}

TEST_CASE("native_tetrahedralization_simplex_and_scale", "[tetrahedralization]")
{
    for(double scale : {1e-5, 1.0, 1e5})
    {
        vector<Vector3> points{{0, 0, 0}, {scale, 0, 0}, {0, scale, 0}, {0, 0, scale}};
        vector<Vector3i> fs{{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};
        auto             surface     = trimesh(points, fs);
        auto             options     = tetrahedralization_default_config();
        options["quality_passes"]    = 0;
        options["refinement_budget"] = 0;
        auto [mesh, report]          = tetrahedralize(surface, options);
        check_volume(surface, mesh, scale * scale * scale / 6, true);
        REQUIRE(mesh.tetrahedra().size() == 1);
        REQUIRE(mesh.vertices().size() == 4);
    }
}

TEST_CASE("native_tetrahedralization_extracted_surface", "[tetrahedralization]")
{
    vector<Vector3>  points{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    vector<Vector4i> cells{{0, 1, 2, 3}};
    auto             volume = tetmesh(points, cells);
    label_surface(volume);
    label_triangle_orient(volume);
    auto surface        = extract_surface(volume);
    auto [mesh, report] = tetrahedralize(surface);
    check_volume(surface, mesh, 1.0 / 6, true);
}
