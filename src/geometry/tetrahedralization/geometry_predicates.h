// Copyright (C) 2026 spiriMirror
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "predicates.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <map>
#include <set>

namespace uipc::geometry::tetrahedralization
{
using Face   = std::array<IndexT, 3>;
using Tet    = std::array<IndexT, 4>;
using Edge   = std::array<IndexT, 2>;
using Points = std::vector<Vector3>;
using Front  = std::map<Face, Face>;

template <size_t N>
inline std::array<IndexT, N> key(std::array<IndexT, N> value)
{
    std::sort(value.begin(), value.end());
    return value;
}

inline std::array<Face, 4> faces(const Tet& t)
{
    return {Face{t[1], t[2], t[3]},
            Face{t[0], t[3], t[2]},
            Face{t[0], t[1], t[3]},
            Face{t[0], t[2], t[1]}};
}

inline bool same_orientation(const Face& a, const Face& b)
{
    for(int i = 0; i < 3; ++i)
        if(a[0] == b[i])
            return a[1] == b[(i + 1) % 3];
    return false;
}

inline Face reversed(Face face)
{
    std::swap(face[1], face[2]);
    return face;
}

struct Box
{
    Vector3 lo = Vector3::Constant(std::numeric_limits<Float>::infinity());
    Vector3 hi = -lo;

    void add(const Vector3& p)
    {
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }

    bool intersects(const Box& other) const
    {
        return (lo.array() <= other.hi.array()).all()
               && (other.lo.array() <= hi.array()).all();
    }

    bool contains(const Vector3& p) const
    {
        return (p.array() >= lo.array()).all() && (p.array() <= hi.array()).all();
    }
};

template <size_t N>
inline Box bounds(const Points& p, const std::array<IndexT, N>& ids)
{
    Box box;
    for(auto id : ids)
        box.add(p[id]);
    return box;
}

inline int projection(const Points& p, const Face& f)
{
    Vector3      n = (p[f[1]] - p[f[0]]).cross(p[f[2]] - p[f[0]]);
    Eigen::Index axis;
    n.cwiseAbs().maxCoeff(&axis);
    return static_cast<int>(axis);
}

inline int projected_orient(const Vector3& a, const Vector3& b, const Vector3& c, int axis)
{
    int i = (axis + 1) % 3, j = (axis + 2) % 3;
    return orient2(a[i], a[j], b[i], b[j], c[i], c[j]);
}

inline bool in_triangle(const Vector3& point, const Points& p, const Face& face, int axis)
{
    int signs[3];
    for(int i = 0; i < 3; ++i)
        signs[i] = projected_orient(p[face[i]], p[face[(i + 1) % 3]], point, axis);
    return (signs[0] >= 0 && signs[1] >= 0 && signs[2] >= 0)
           || (signs[0] <= 0 && signs[1] <= 0 && signs[2] <= 0);
}

inline bool forbidden_planar_edges(const Points& p, Edge a, Edge b, int axis)
{
    if(key(a) == key(b))
        return false;
    for(int i = 0; i < 2; ++i)
        for(int j = 0; j < 2; ++j)
            if(a[i] == b[j])
            {
                if(projected_orient(p[a[i]], p[a[1 - i]], p[b[1 - j]], axis) != 0)
                    return false;
                return (p[a[1 - i]] - p[a[i]]).dot(p[b[1 - j]] - p[a[i]]) > 0;
            }
    if(!bounds(p, a).intersects(bounds(p, b)))
        return false;
    int aa = projected_orient(p[a[0]], p[a[1]], p[b[0]], axis);
    int ab = projected_orient(p[a[0]], p[a[1]], p[b[1]], axis);
    int ba = projected_orient(p[b[0]], p[b[1]], p[a[0]], axis);
    int bb = projected_orient(p[b[0]], p[b[1]], p[a[1]], axis);
    return aa * ab <= 0 && ba * bb <= 0;
}

inline bool forbidden_edge_face(const Points& p, Edge edge, Face face)
{
    int a = orient3(p[face[0]], p[face[1]], p[face[2]], p[edge[0]]);
    int b = orient3(p[face[0]], p[face[1]], p[face[2]], p[edge[1]]);
    if(a * b > 0)
        return false;
    auto shared = [&](IndexT id)
    { return std::find(face.begin(), face.end(), id) != face.end(); };
    int axis = projection(p, face);
    if(a == 0 && b == 0)
    {
        for(auto id : edge)
            if(!shared(id) && in_triangle(p[id], p, face, axis))
                return true;
        for(int i = 0; i < 3; ++i)
            if(forbidden_planar_edges(p, edge, {face[i], face[(i + 1) % 3]}, axis))
                return true;
        return false;
    }
    if(a == 0)
        return !shared(edge[0]) && in_triangle(p[edge[0]], p, face, axis);
    if(b == 0)
        return !shared(edge[1]) && in_triangle(p[edge[1]], p, face, axis);
    int signs[3];
    for(int i = 0; i < 3; ++i)
        signs[i] = orient3(p[edge[0]], p[edge[1]], p[face[i]], p[face[(i + 1) % 3]]);
    return (signs[0] >= 0 && signs[1] >= 0 && signs[2] >= 0)
           || (signs[0] <= 0 && signs[1] <= 0 && signs[2] <= 0);
}

inline bool forbidden_faces(const Points& p, Face a, Face b)
{
    if(key(a) == key(b) || !bounds(p, a).intersects(bounds(p, b)))
        return false;
    int    shared   = 0;
    IndexT opposite = b[0];
    for(auto id : b)
        if(std::find(a.begin(), a.end(), id) != a.end())
            ++shared;
        else
            opposite = id;
    if(shared == 2 && orient3(p[a[0]], p[a[1]], p[a[2]], p[opposite]) != 0)
        return false;
    for(int i = 0; i < 3; ++i)
        if(forbidden_edge_face(p, {a[i], a[(i + 1) % 3]}, b)
           || forbidden_edge_face(p, {b[i], b[(i + 1) % 3]}, a))
            return true;
    return false;
}

inline bool in_tet(const Points& p, Tet t, const Vector3& point)
{
    for(auto f : faces(t))
        if(orient3(p[f[0]], p[f[1]], p[f[2]], point) > 0)
            return false;
    return true;
}

inline double six_volume(const Points& p, Tet t)
{
    return (p[t[1]] - p[t[0]]).dot((p[t[2]] - p[t[0]]).cross(p[t[3]] - p[t[0]]));
}

inline double quality(const Points& p, Tet t)
{
    double sum = 0;
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < i; ++j)
            sum += (p[t[i]] - p[t[j]]).squaredNorm();
    return sum > 0 ? 12 * std::pow(std::abs(six_volume(p, t)) / 2, 2.0 / 3.0) / sum : 0;
}

inline bool positive_tet(const Points& p, Tet& t)
{
    int direction = orient3(p[t[0]], p[t[1]], p[t[2]], p[t[3]]);
    if(direction < 0)
        std::swap(t[1], t[2]);
    return direction != 0;
}
}  // namespace uipc::geometry::tetrahedralization
