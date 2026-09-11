// Copyright (C) 2026 spiriMirror
// SPDX-License-Identifier: Apache-2.0
#include <uipc/geometry/tetrahedralization.h>
#include <uipc/geometry/utils/factory.h>
#include <uipc/geometry/utils/label_surface.h>
#include <uipc/geometry/utils/label_triangle_orient.h>
#include <uipc/common/log.h>
#include <uipc/builtin/attribute_name.h>
#include "geometry_predicates.h"
#include <Eigen/Dense>
#include <chrono>
#include <numeric>
#include <unordered_set>

namespace uipc::geometry::tetrahedralization
{
struct Construction
{
    Points            points;
    std::vector<Tet>  cells;
    std::vector<Face> boundary;
    size_t            boundary_vertices = 0;
    size_t            search_nodes      = 0;
    size_t            search_rounds     = 0;
    std::string       method;
};

static void validate_surface(const Points& p, std::vector<Face>& boundary)
{
    UIPC_ASSERT_THROW(p.size() >= 4 && boundary.size() >= 4,
                      "Tetrahedralization needs a closed triangle surface.");
    std::set<std::array<Float, 3>> coordinates;
    for(const auto& point : p)
    {
        UIPC_ASSERT_THROW(point.allFinite(), "Non-finite tetrahedralization input coordinate.");
        bool unique_position =
            coordinates.insert({point.x(), point.y(), point.z()}).second;
        UIPC_ASSERT_THROW(unique_position, "Coincident input vertices must be welded before tetrahedralization.");
    }
    std::set<Face>                    unique;
    std::map<Edge, std::vector<bool>> edges;
    std::vector<bool>                 used(p.size(), false);
    for(auto face : boundary)
    {
        for(auto id : face)
        {
            UIPC_ASSERT_THROW(id >= 0 && static_cast<size_t>(id) < p.size(),
                              "Surface index out of range.");
            used[id] = true;
        }
        UIPC_ASSERT_THROW(face[0] != face[1] && face[1] != face[2] && face[2] != face[0],
                          "Surface triangle has repeated vertex indices.");
        bool nonzero = false;
        for(int axis = 0; axis < 3; ++axis)
            nonzero |= projected_orient(p[face[0]], p[face[1]], p[face[2]], axis) != 0;
        UIPC_ASSERT_THROW(nonzero, "Degenerate input surface triangle.");
        UIPC_ASSERT_THROW(unique.insert(key(face)).second, "Duplicate input surface triangle.");
        for(int i = 0; i < 3; ++i)
        {
            Edge edge{face[i], face[(i + 1) % 3]};
            edges[key(edge)].push_back(edge[0] < edge[1]);
        }
    }
    for(auto&& [edge, signs] : edges)
        UIPC_ASSERT_THROW(signs.size() == 2 && signs[0] != signs[1],
                          "Input must be closed with consistently oriented manifold edges: ({}, {}).",
                          edge[0],
                          edge[1]);
    bool all_used =
        std::all_of(used.begin(), used.end(), [](bool x) { return x; });
    UIPC_ASSERT_THROW(all_used, "Remove vertices unused by the input surface.");
    // An embedded boundary is a precondition, not an optimization failure.
    std::vector<Box> boxes;
    for(auto face : boundary)
        boxes.push_back(bounds(p, face));
    Box surface_box;
    for(const auto& point : p)
        surface_box.add(point);
    Eigen::Index sweep_axis;
    (surface_box.hi - surface_box.lo).maxCoeff(&sweep_axis);
    std::vector<size_t> order(boundary.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(),
              order.end(),
              [&](size_t a, size_t b)
              { return boxes[a].lo[sweep_axis] < boxes[b].lo[sweep_axis]; });
    for(size_t a = 0; a < order.size(); ++a)
        for(size_t b = a + 1; b < order.size(); ++b)
        {
            size_t i = order[a], j = order[b];
            if(boxes[j].lo[sweep_axis] > boxes[i].hi[sweep_axis])
                break;
            if(boxes[i].intersects(boxes[j]))
                UIPC_ASSERT_THROW(!forbidden_faces(p, boundary[i], boundary[j]),
                                  "Input surface self-intersects at triangles {} and {}.",
                                  i,
                                  j);
        }
    double  volume = 0;
    Vector3 origin = p.front();
    for(auto face : boundary)
        volume += (p[face[0]] - origin).dot((p[face[1]] - origin).cross(p[face[2]] - origin));
    UIPC_ASSERT_THROW(std::isfinite(volume) && volume != 0,
                      "Input encloses no finite nonzero volume.");
    if(volume < 0)
        for(auto& face : boundary)
            face = reversed(face);
}

static bool kernel(const Points& p, const std::vector<Face>& boundary, Vector3& candidate)
{
    auto accepts = [&](const Vector3& point)
    {
        if(!point.allFinite())
            return false;
        for(auto f : boundary)
            if(orient3(p[f[0]], p[f[1]], p[f[2]], point) >= 0)
                return false;
        return true;
    };
    std::set<IndexT> vertices;
    Box              box;
    for(auto f : boundary)
        for(auto id : f)
            vertices.insert(id);
    candidate.setZero();
    for(auto id : vertices)
    {
        candidate += p[id];
        box.add(p[id]);
    }
    candidate /= static_cast<double>(vertices.size());
    if(accepts(candidate))
        return true;
    Vector3 initial = (box.lo + box.hi) * 0.5;
    if(accepts(initial))
    {
        candidate = initial;
        return true;
    }
    // Half-space projection seeks an interior visibility kernel. Acceptance is
    // certified by orientation predicates; a heuristic miss uses the general front.
    std::vector<Vector3> normals;
    std::vector<double>  offsets;
    for(auto face : boundary)
    {
        Vector3 normal = (p[face[1]] - p[face[0]]).cross(p[face[2]] - p[face[0]]);
        double magnitude = normal.cwiseAbs().maxCoeff();
        if(!std::isfinite(magnitude) || magnitude == 0)
            return false;
        normal /= magnitude;
        normal.normalize();
        normals.push_back(normal);
        offsets.push_back(normal.dot(p[face[0]]));
    }
    double diagonal = (box.hi - box.lo).norm();
    for(double relative : {1e-2, 1e-4, 1e-7})
    {
        candidate     = initial;
        double margin = diagonal * relative;
        for(int iteration = 0; iteration < 256; ++iteration)
        {
            for(size_t i = 0; i < normals.size(); ++i)
            {
                double violation = normals[i].dot(candidate) - offsets[i] + margin;
                if(violation > 0)
                    candidate -= violation * normals[i];
            }
            if(accepts(candidate))
                return true;
        }
    }
    return false;
}

static bool admissible(const Points& p, const Front& front, Tet cell)
{
    if(orient3(p[cell[0]], p[cell[1]], p[cell[2]], p[cell[3]]) <= 0)
        return false;
    Box  box        = bounds(p, cell);
    auto cell_faces = faces(cell);
    for(auto face : cell_faces)
    {
        auto found = front.find(key(face));
        if(found != front.end() && !same_orientation(face, found->second))
            return false;
    }
    for(auto&& [unused, face] : front)
    {
        if(!box.intersects(bounds(p, face)))
            continue;
        // No unresolved boundary vertex may become an interior or hanging node.
        for(auto id : face)
            if(std::find(cell.begin(), cell.end(), id) == cell.end()
               && box.contains(p[id]) && in_tet(p, cell, p[id]))
                return false;
        for(auto cell_face : cell_faces)
            if(forbidden_faces(p, cell_face, face))
                return false;
    }
    return true;
}

struct Change
{
    std::vector<Face> added;
    std::vector<Face> removed;
};

static Change insert(Front& front, Tet cell)
{
    Change change;
    for(auto face : faces(cell))
    {
        auto k     = key(face);
        auto found = front.find(k);
        if(found != front.end())
        {
            change.removed.push_back(found->second);
            front.erase(found);
        }
        else
        {
            front.emplace(k, reversed(face));
            change.added.push_back(k);
        }
    }
    return change;
}

static void undo(Front& front, const Change& change)
{
    for(auto k : change.added)
        front.erase(k);
    for(auto f : change.removed)
        front.emplace(key(f), f);
}

struct SearchFrame
{
    Face                base;
    std::vector<IndexT> apices;
    size_t              next = 0;
    Change              incoming;
};

static SearchFrame choices(const Points& p, const Front& front, size_t limit, size_t point_count)
{
    SearchFrame frame;
    frame.base = front.begin()->second;
    // Smaller fronts usually arise from the most constrained original corners.
    const auto                             f = frame.base;
    std::vector<std::pair<double, IndexT>> ordered;
    ordered.reserve(point_count);
    for(size_t i = 0; i < point_count; ++i)
    {
        auto id = static_cast<IndexT>(i);
        if(std::find(f.begin(), f.end(), id) != f.end()
           || orient3(p[f[0]], p[f[1]], p[f[2]], p[id]) >= 0)
            continue;
        Tet cell{f[0], f[2], f[1], id};
        int consumed = 0;
        for(auto face : faces(cell))
        {
            auto found = front.find(key(face));
            consumed += found != front.end() && same_orientation(face, found->second);
        }
        double score = 4 * consumed + quality(p, cell);
        ordered.emplace_back(-score, id);
    }
    std::sort(ordered.begin(), ordered.end());
    if(limit && ordered.size() > limit)
        ordered.resize(limit);
    for(auto [score, id] : ordered)
        frame.apices.push_back(id);
    return frame;
}

static bool search(Construction& mesh, size_t budget, size_t candidate_limit, size_t point_count)
{
    Front front;
    for(auto face : mesh.boundary)
        front.emplace(key(face), face);
    mesh.cells.clear();
    std::vector<SearchFrame> stack;
    stack.push_back(choices(mesh.points, front, candidate_limit, point_count));
    size_t visited = 0;
    while(!stack.empty())
    {
        if(++visited > budget)
            break;
        auto& frame = stack.back();
        if(frame.next == frame.apices.size())
        {
            if(stack.size() > 1)
            {
                undo(front, frame.incoming);
                mesh.cells.pop_back();
            }
            stack.pop_back();
            continue;
        }
        auto id = frame.apices[frame.next++];
        Tet  cell{frame.base[0], frame.base[2], frame.base[1], id};
        if(!admissible(mesh.points, front, cell))
            continue;
        auto change = insert(front, cell);
        mesh.cells.push_back(cell);
        if(front.empty())
        {
            mesh.search_nodes += visited;
            return true;
        }
        auto child = choices(mesh.points, front, candidate_limit, point_count);
        child.incoming = std::move(change);
        stack.push_back(std::move(child));
    }
    mesh.search_nodes += visited;
    mesh.cells.clear();
    return false;
}

static double radical_inverse(size_t value, size_t base)
{
    double result = 0, factor = 1.0 / static_cast<double>(base);
    while(value)
    {
        result += static_cast<double>(value % base) * factor;
        value /= base;
        factor /= static_cast<double>(base);
    }
    return result;
}

static void compact(Construction& mesh)
{
    std::vector<bool> used(mesh.points.size(), false);
    for(auto cell : mesh.cells)
        for(auto id : cell)
            used[id] = true;
    std::vector<IndexT> mapping(mesh.points.size(), -1);
    Points              result;
    for(size_t i = 0; i < mesh.points.size(); ++i)
    {
        if(i < mesh.boundary_vertices)
            UIPC_ASSERT_THROW(used[i], "Internal error: mesher lost an input surface vertex.");
        if(used[i])
        {
            mapping[i] = static_cast<IndexT>(result.size());
            result.push_back(mesh.points[i]);
        }
    }
    for(auto& cell : mesh.cells)
        for(auto& id : cell)
            id = mapping[id];
    mesh.points = std::move(result);
}

static void conservative(Construction& mesh)
{
    if(mesh.boundary_vertices == 4 && mesh.boundary.size() == 4)
    {
        Tet cell{0, 1, 2, 3};
        if(positive_tet(mesh.points, cell))
        {
            mesh.cells.push_back(cell);
            mesh.method = "single_tetrahedron";
            return;
        }
    }
    Vector3 center;
    if(kernel(mesh.points, mesh.boundary, center))
    {
        auto id = static_cast<IndexT>(mesh.points.size());
        mesh.points.push_back(center);
        for(auto f : mesh.boundary)
            mesh.cells.push_back({f[0], f[2], f[1], id});
        mesh.method = "visibility_kernel";
        return;
    }
    mesh.method = "constrained_advancing_front";
    if(search(mesh, 20000, 48, mesh.points.size()))
        return;
    Box box;
    for(auto point : mesh.points)
        box.add(point);
    size_t              sequence = 1;
    size_t              budget   = 40000;
    std::vector<size_t> candidate_prefixes;
    // A failed quality-oriented search is not a failed meshing operation. Retry
    // the constructive front with interior Steiner candidates and a growing
    // backtracking budget. The candidate set is nested; original boundary faces
    // are never split, moved, or replaced. No optimization budget bounds this loop.
    for(size_t round = 0;; ++round)
    {
        ++mesh.search_rounds;
        size_t batch = round == 0 ? 0 : std::min<size_t>(16 + 8 * round, 256);
        for(size_t i = 0; i < batch; ++i, ++sequence)
        {
            Vector3 point;
            for(int axis = 0; axis < 3; ++axis)
            {
                const size_t bases[3] = {2, 3, 5};
                point[axis]           = box.lo[axis]
                              + (box.hi[axis] - box.lo[axis])
                                    * radical_inverse(sequence, bases[axis]);
            }
            if(std::none_of(mesh.points.begin(),
                            mesh.points.end(),
                            [&](const Vector3& old)
                            { return (old.array() == point.array()).all(); }))
                mesh.points.push_back(point);
        }
        UIPC_INFO_WITH_LOCATION("Tetrahedralization conservative search: round={}, candidates={}, search_budget={}.",
                                round,
                                mesh.points.size(),
                                budget);
        candidate_prefixes.push_back(mesh.points.size());
        // Revisit earlier finite candidate sets too: later sample points must
        // not indefinitely postpone a solution on an already sufficient set.
        for(auto prefix = candidate_prefixes.rbegin();
            prefix != candidate_prefixes.rend();
            ++prefix)
        {
            if(search(mesh, budget, 0, *prefix))
            {
                compact(mesh);
                return;
            }
        }
        if(budget < std::numeric_limits<size_t>::max() / 2)
            budget *= 2;
    }
}

static double minimum_quality(const Points& p, const std::vector<Tet>& cells)
{
    double result = 1;
    for(auto cell : cells)
        result = std::min(result, quality(p, cell));
    return result;
}

static std::set<Edge> boundary_edges(const std::vector<Face>& boundary)
{
    std::set<Edge> result;
    for(auto face : boundary)
        for(int i = 0; i < 3; ++i)
            result.insert(key(Edge{face[i], face[(i + 1) % 3]}));
    return result;
}

static void refine_boundary(Construction& mesh, double target)
{
    // Conforming midpoint refinement, permitted only when preserve_surface=false.
    // Refine every face in a round so no hanging edge vertices are introduced.
    for(int round = 0; round < 3; ++round)
    {
        double longest = 0;
        for(auto edge : boundary_edges(mesh.boundary))
            longest =
                std::max(longest, (mesh.points[edge[0]] - mesh.points[edge[1]]).norm());
        if(longest <= 1.75 * target)
            break;
        std::map<Edge, IndexT> midpoints;
        auto                   midpoint = [&](IndexT a, IndexT b)
        {
            Edge edge  = key(Edge{a, b});
            auto found = midpoints.find(edge);
            if(found != midpoints.end())
                return found->second;
            auto id = static_cast<IndexT>(mesh.points.size());
            mesh.points.push_back((mesh.points[a] + mesh.points[b]) * 0.5);
            midpoints.emplace(edge, id);
            return id;
        };
        std::vector<Face> refined;
        for(auto f : mesh.boundary)
        {
            auto a = midpoint(f[0], f[1]), b = midpoint(f[1], f[2]),
                 c = midpoint(f[2], f[0]);
            refined.push_back({f[0], a, c});
            refined.push_back({a, f[1], b});
            refined.push_back({c, b, f[2]});
            refined.push_back({a, b, c});
        }
        mesh.boundary = std::move(refined);
    }
    mesh.boundary_vertices = mesh.points.size();
}

static bool patch_boundary(const std::vector<Tet>& cells, Front& boundary)
{
    for(auto cell : cells)
        for(auto face : faces(cell))
        {
            auto found = boundary.find(key(face));
            if(found == boundary.end())
                boundary.emplace(key(face), face);
            else
            {
                if(same_orientation(found->second, face))
                    return false;
                boundary.erase(found);
            }
        }
    return true;
}

static size_t improve_faces(Construction& mesh)
{
    std::map<Face, std::vector<size_t>> adjacent;
    std::set<Edge>                      existing_edges;
    for(size_t i = 0; i < mesh.cells.size(); ++i)
    {
        for(auto face : faces(mesh.cells[i]))
            adjacent[key(face)].push_back(i);
        for(int a = 0; a < 4; ++a)
            for(int b = 0; b < a; ++b)
                existing_edges.insert(key(Edge{mesh.cells[i][a], mesh.cells[i][b]}));
    }
    std::set<size_t> removed;
    std::vector<Tet> added;
    size_t           accepted = 0;
    for(auto&& [face, pair] : adjacent)
    {
        if(pair.size() != 2 || removed.contains(pair[0]) || removed.contains(pair[1]))
            continue;
        IndexT apices[2] = {-1, -1};
        for(int side = 0; side < 2; ++side)
            for(auto id : mesh.cells[pair[side]])
                if(std::find(face.begin(), face.end(), id) == face.end())
                    apices[side] = id;
        if(existing_edges.contains(key(Edge{apices[0], apices[1]})))
            continue;
        std::vector<Tet> old{mesh.cells[pair[0]], mesh.cells[pair[1]]};
        std::vector<Tet> next;
        double           before = minimum_quality(mesh.points, old);
        bool             valid  = true;
        for(int i = 0; i < 3; ++i)
        {
            Tet cell{apices[0], apices[1], face[i], face[(i + 1) % 3]};
            valid &= positive_tet(mesh.points, cell);
            next.push_back(cell);
        }
        if(!valid || minimum_quality(mesh.points, next) <= before * 1.001)
            continue;
        Front old_boundary, new_boundary;
        if(!patch_boundary(old, old_boundary) || !patch_boundary(next, new_boundary)
           || old_boundary.size() != new_boundary.size())
            continue;
        for(auto&& [key, oriented] : old_boundary)
        {
            auto found = new_boundary.find(key);
            valid &= found != new_boundary.end()
                     && same_orientation(oriented, found->second);
        }
        if(!valid)
            continue;
        removed.insert(pair[0]);
        removed.insert(pair[1]);
        added.insert(added.end(), next.begin(), next.end());
        existing_edges.insert(key(Edge{apices[0], apices[1]}));
        ++accepted;
    }
    if(accepted)
    {
        std::vector<Tet> result;
        for(size_t i = 0; i < mesh.cells.size(); ++i)
            if(!removed.contains(i))
                result.push_back(mesh.cells[i]);
        result.insert(result.end(), added.begin(), added.end());
        mesh.cells = std::move(result);
    }
    return accepted;
}

static size_t improve(Construction& mesh, double target, size_t passes, size_t refinement_budget)
{
    auto   locked_edges = boundary_edges(mesh.boundary);
    size_t accepted     = 0;
    for(size_t step = 0; step < refinement_budget; ++step)
    {
        std::map<Edge, std::vector<size_t>> incidence;
        for(size_t i = 0; i < mesh.cells.size(); ++i)
            for(int a = 0; a < 4; ++a)
                for(int b = 0; b < a; ++b)
                    incidence[key(Edge{mesh.cells[i][a], mesh.cells[i][b]})].push_back(i);
        std::vector<std::pair<double, Edge>> ordered;
        for(auto&& [edge, adjacent] : incidence)
            if(!locked_edges.contains(edge))
                ordered.emplace_back(
                    -(mesh.points[edge[0]] - mesh.points[edge[1]]).squaredNorm(), edge);
        std::sort(ordered.begin(), ordered.end());
        bool changed = false;
        for(auto [negative_length, edge] : ordered)
        {
            if(-negative_length <= target * target * 1.44)
                break;
            auto    id    = static_cast<IndexT>(mesh.points.size());
            Vector3 point = (mesh.points[edge[0]] + mesh.points[edge[1]]) * 0.5;
            if((point.array() == mesh.points[edge[0]].array()).all()
               || (point.array() == mesh.points[edge[1]].array()).all())
                continue;
            mesh.points.push_back(point);
            std::vector<Tet> replacements;
            double           before = 1, after = 1;
            bool             valid = true;
            for(auto index : incidence[edge])
            {
                auto old = mesh.cells[index];
                before   = std::min(before, quality(mesh.points, old));
                for(auto endpoint : edge)
                {
                    Tet next = old;
                    std::replace(next.begin(), next.end(), endpoint, id);
                    valid &= positive_tet(mesh.points, next);
                    after = std::min(after, quality(mesh.points, next));
                    replacements.push_back(next);
                }
            }
            bool improves_quality    = after > before * 1.001;
            bool improves_resolution = -negative_length > 4 * target * target
                                       && after >= std::min(0.05, before * 0.5);
            if(!valid || (!improves_quality && !improves_resolution))
            {
                mesh.points.pop_back();
                continue;
            }
            std::set<size_t> removed(incidence[edge].begin(), incidence[edge].end());
            std::vector<Tet> cells;
            for(size_t i = 0; i < mesh.cells.size(); ++i)
                if(!removed.contains(i))
                    cells.push_back(mesh.cells[i]);
            cells.insert(cells.end(), replacements.begin(), replacements.end());
            mesh.cells = std::move(cells);
            ++accepted;
            changed = true;
            break;
        }
        if(!changed)
            break;
    }
    for(size_t pass = 0; pass < passes; ++pass)
    {
        accepted += improve_faces(mesh);
        std::vector<std::vector<size_t>> adjacent(mesh.points.size());
        for(size_t i = 0; i < mesh.cells.size(); ++i)
            for(auto id : mesh.cells[i])
                adjacent[id].push_back(i);
        for(size_t vertex = mesh.boundary_vertices; vertex < mesh.points.size(); ++vertex)
        {
            std::set<IndexT> neighbors;
            double           before = 1;
            for(auto cell : adjacent[vertex])
            {
                before = std::min(before, quality(mesh.points, mesh.cells[cell]));
                for(auto id : mesh.cells[cell])
                    if(static_cast<size_t>(id) != vertex)
                        neighbors.insert(id);
            }
            if(neighbors.empty())
                continue;
            Vector3 mean = Vector3::Zero();
            for(auto id : neighbors)
                mean += mesh.points[id];
            mean /= static_cast<double>(neighbors.size());
            Vector3 original = mesh.points[vertex];
            for(double fraction : {1.0, 0.5, 0.25})
            {
                mesh.points[vertex] = original + fraction * (mean - original);
                bool   valid        = true;
                double after        = 1;
                for(auto index : adjacent[vertex])
                {
                    auto cell = mesh.cells[index];
                    valid &= orient3(mesh.points[cell[0]],
                                     mesh.points[cell[1]],
                                     mesh.points[cell[2]],
                                     mesh.points[cell[3]])
                             > 0;
                    after = std::min(after, quality(mesh.points, cell));
                }
                if(valid && after > before * 1.0001)
                {
                    ++accepted;
                    break;
                }
                mesh.points[vertex] = original;
            }
        }
    }
    return accepted;
}

static double audit(const Construction& mesh)
{
    std::map<Face, std::pair<Face, int>> boundary;
    double                               volume = 0;
    for(auto cell : mesh.cells)
    {
        UIPC_ASSERT_THROW(orient3(mesh.points[cell[0]],
                                  mesh.points[cell[1]],
                                  mesh.points[cell[2]],
                                  mesh.points[cell[3]])
                              > 0,
                          "Internal tetrahedralization error: non-positive cell.");
        volume += six_volume(mesh.points, cell) / 6;
        for(auto f : faces(cell))
        {
            auto [i, inserted] = boundary.emplace(key(f), std::make_pair(f, 1));
            if(!inserted)
            {
                UIPC_ASSERT_THROW(
                    i->second.second == 1 && !same_orientation(i->second.first, f),
                    "Internal tetrahedralization error: overlapping or non-manifold face.");
                ++i->second.second;
            }
        }
    }
    std::set<Face> actual, expected;
    for(auto&& [f, record] : boundary)
        if(record.second == 1)
            actual.insert(f);
    double  surface_volume = 0;
    Vector3 origin         = mesh.points.front();
    for(auto f : mesh.boundary)
    {
        expected.insert(key(f));
        auto found = boundary.find(key(f));
        UIPC_ASSERT_THROW(found != boundary.end() && found->second.second == 1
                              && same_orientation(f, found->second.first),
                          "Internal tetrahedralization error: oriented boundary mismatch.");
        surface_volume +=
            (mesh.points[f[0]] - origin)
                .dot((mesh.points[f[1]] - origin).cross(mesh.points[f[2]] - origin))
            / 6;
    }
    UIPC_ASSERT_THROW(actual == expected, "Internal tetrahedralization error: boundary mismatch.");
    UIPC_ASSERT_THROW(std::abs(volume - surface_volume)
                          <= 1e-9 * std::max(std::abs(volume), std::abs(surface_volume)),
                      "Internal tetrahedralization error: volume mismatch ({} vs {}).",
                      volume,
                      surface_volume);
    return volume;
}
}  // namespace uipc::geometry::tetrahedralization

namespace uipc::geometry
{
Json tetrahedralization_default_config()
{
    return Json{{"preserve_surface", true},
                {"target_edge_length", 0.0},
                {"quality_passes", 4},
                {"refinement_budget", 256}};
}

std::pair<SimplicialComplex, Json> tetrahedralize(const SimplicialComplex& surface,
                                                  const Json& config)
{
    using namespace tetrahedralization;
    auto options = tetrahedralization_default_config();
    for(auto i = config.begin(); i != config.end(); ++i)
    {
        UIPC_ASSERT_THROW(options.contains(i.key()),
                          "Unknown tetrahedralization option '{}'.",
                          i.key());
        options[i.key()] = i.value();
    }
    UIPC_ASSERT_THROW(surface.dim() == 2, "Tetrahedralization input must be a triangle surface.");
    UIPC_ASSERT_THROW(options["preserve_surface"].is_boolean(),
                      "preserve_surface must be boolean.");
    bool   preserve   = options["preserve_surface"].get<bool>();
    double target     = options["target_edge_length"].get<double>();
    auto   passes     = options["quality_passes"].get<int>();
    auto   refinement = options["refinement_budget"].get<int>();
    UIPC_ASSERT_THROW(std::isfinite(target) && target >= 0 && passes >= 0
                          && passes <= 100 && refinement >= 0,
                      "Invalid tetrahedralization quality options.");
    auto         started = std::chrono::steady_clock::now();
    Construction mesh;
    for(auto point : surface.positions().view())
        mesh.points.push_back(point);
    const auto original    = mesh.points;
    auto       orientation = surface.triangles().find<IndexT>(builtin::orient);
    auto       input_triangles = surface.triangles().topo().view();
    for(size_t i = 0; i < input_triangles.size(); ++i)
    {
        auto f = input_triangles[i];
        Face face{f[0], f[1], f[2]};
        if(orientation && orientation->view()[i] < 0)
            face = reversed(face);
        mesh.boundary.push_back(face);
    }
    validate_surface(mesh.points, mesh.boundary);
    mesh.boundary_vertices    = mesh.points.size();
    const auto original_faces = mesh.boundary;
    if(target == 0)
    {
        std::vector<double> lengths;
        for(auto edge : boundary_edges(mesh.boundary))
            lengths.push_back((mesh.points[edge[0]] - mesh.points[edge[1]]).norm());
        std::nth_element(lengths.begin(),
                         lengths.begin() + lengths.size() / 2,
                         lengths.end());
        target = lengths[lengths.size() / 2];
    }
    if(!preserve)
        refine_boundary(mesh, target);
    conservative(mesh);
    double before = minimum_quality(mesh.points, mesh.cells);
    // Establish and certify the conservative mesh before any quality operation.
    audit(mesh);
    auto   baseline              = mesh;
    size_t accepted              = 0;
    bool   optimization_reverted = false;
    double volume                = 0;
    try
    {
        accepted =
            improve(mesh, target, static_cast<size_t>(passes), static_cast<size_t>(refinement));
        volume = audit(mesh);
    }
    catch(const std::exception& error)
    {
        UIPC_WARN_WITH_LOCATION("Tetrahedral quality optimization reverted to its certified conservative mesh: {}",
                                error.what());
        mesh                  = std::move(baseline);
        accepted              = 0;
        optimization_reverted = true;
        volume                = audit(mesh);
    }
    for(size_t i = 0; i < original.size(); ++i)
        UIPC_ASSERT_THROW((mesh.points[i].array() == original[i].array()).all(),
                          "Internal tetrahedralization error: moved an original vertex.");
    if(preserve)
        UIPC_ASSERT_THROW(mesh.boundary == original_faces,
                          "Internal tetrahedralization error: modified protected surface.");
    vector<Vector3>  positions(mesh.points.begin(), mesh.points.end());
    vector<Vector4i> cells;
    cells.reserve(mesh.cells.size());
    for(auto t : mesh.cells)
        cells.push_back(Vector4i{t[0], t[1], t[2], t[3]});
    auto result = tetmesh(positions, cells);
    label_surface(result);
    label_triangle_orient(result);
    auto ids     = result.vertices().create<IndexT>("input_vertex_id", -1);
    auto id_view = view(*ids);
    for(size_t i = 0; i < original.size(); ++i)
        id_view[i] = static_cast<IndexT>(i);
    auto face_ids = result.triangles().create<IndexT>("input_triangle_id", -1);
    auto face_id_view = view(*face_ids);
    std::map<Face, IndexT> original_face_ids;
    auto                   source_faces = surface.triangles().topo().view();
    for(size_t i = 0; i < source_faces.size(); ++i)
    {
        auto face = source_faces[i];
        original_face_ids.emplace(key(Face{face[0], face[1], face[2]}),
                                  static_cast<IndexT>(i));
    }
    auto output_faces = result.triangles().topo().view();
    for(size_t i = 0; i < output_faces.size(); ++i)
    {
        auto face = output_faces[i];
        auto found = original_face_ids.find(key(Face{face[0], face[1], face[2]}));
        if(found != original_face_ids.end())
            face_id_view[i] = found->second;
    }
    Json report{{"method", mesh.method},
                {"preserve_surface", preserve},
                {"input_vertices", original.size()},
                {"vertices", positions.size()},
                {"tetrahedra", cells.size()},
                {"boundary_triangles", mesh.boundary.size()},
                {"conservative_min_quality", before},
                {"min_quality", minimum_quality(mesh.points, mesh.cells)},
                {"target_edge_length", target},
                {"accepted_improvements", accepted},
                {"optimization_reverted", optimization_reverted},
                {"conservative_search_nodes", mesh.search_nodes},
                {"conservative_search_rounds", mesh.search_rounds},
                {"volume", volume},
                {"boundary_verified", true},
                {"oriented_boundary_verified", true},
                {"elapsed_seconds",
                 std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
                     .count()}};
    return {std::move(result), std::move(report)};
}
}  // namespace uipc::geometry
