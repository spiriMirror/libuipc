// Copyright (C) 2026 spiriMirror
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <uipc/common/type_define.h>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <vector>

namespace uipc::geometry::tetrahedralization
{
// Floating expansions are used only when the fast determinant's error bound
// cannot certify its sign. Each product residual uses correctly rounded fma.
// This implementation must be compiled without floating-point reassociation.
using Expansion = std::vector<double>;

inline void grow(Expansion& values, double value)
{
    Expansion result;
    result.reserve(values.size() + 1);
    for(double x : values)
    {
        double sum       = value + x;
        double virtual_x = sum - value;
        double error     = (value - (sum - virtual_x)) + (x - virtual_x);
        if(error != 0.0)
            result.push_back(error);
        value = sum;
    }
    if(value != 0.0 || result.empty())
        result.push_back(value);
    values = std::move(result);
}

inline Expansion difference(double a, double b)
{
    double    hi        = a - b;
    double    virtual_b = a - hi;
    double    lo        = (a - (hi + virtual_b)) + (virtual_b - b);
    Expansion r;
    if(lo != 0.0)
        r.push_back(lo);
    if(hi != 0.0 || r.empty())
        r.push_back(hi);
    return r;
}

inline Expansion add(Expansion a, const Expansion& b, double sign = 1.0)
{
    for(double x : b)
        grow(a, sign * x);
    return a;
}

inline Expansion multiply(const Expansion& a, const Expansion& b)
{
    Expansion r;
    for(double x : a)
        for(double y : b)
        {
            double product = x * y;
            double error   = std::fma(x, y, -product);
            if(error != 0.0)
                grow(r, error);
            if(product != 0.0)
                grow(r, product);
        }
    return r;
}

inline int sign(const Expansion& value)
{
    for(auto i = value.rbegin(); i != value.rend(); ++i)
        if(*i != 0.0)
            return *i > 0.0 ? 1 : -1;
    return 0;
}

inline int orient2(double ax, double ay, double bx, double by, double cx, double cy)
{
    double u   = (bx - ax) * (cy - ay);
    double v   = (by - ay) * (cx - ax);
    double det = u - v;
    double error =
        32 * std::numeric_limits<double>::epsilon() * (std::abs(u) + std::abs(v));
    if(std::abs(det) > error)
        return det > 0 ? 1 : -1;
    return sign(add(multiply(difference(bx, ax), difference(cy, ay)),
                    multiply(difference(by, ay), difference(cx, ax)),
                    -1.0));
}

inline int orient3(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d)
{
    Vector3 u = b - a, v = c - a, w = d - a;
    double  det = u.dot(v.cross(w));
    double  permanent =
        std::abs(u.x()) * (std::abs(v.y() * w.z()) + std::abs(v.z() * w.y()))
        + std::abs(u.y()) * (std::abs(v.z() * w.x()) + std::abs(v.x() * w.z()))
        + std::abs(u.z()) * (std::abs(v.x() * w.y()) + std::abs(v.y() * w.x()));
    if(std::abs(det) > 128 * std::numeric_limits<double>::epsilon() * permanent)
        return det > 0 ? 1 : -1;
    Expansion ex[3], ey[3], ez[3];
    for(int i = 0; i < 3; ++i)
    {
        ex[i] = difference(b[i], a[i]);
        ey[i] = difference(c[i], a[i]);
        ez[i] = difference(d[i], a[i]);
    }
    Expansion result;
    for(int i = 0; i < 3; ++i)
    {
        int  j = (i + 1) % 3, k = (i + 2) % 3;
        auto minor = add(multiply(ey[j], ez[k]), multiply(ey[k], ez[j]), -1.0);
        result     = add(std::move(result), multiply(ex[i], minor));
    }
    return sign(result);
}
}  // namespace uipc::geometry::tetrahedralization
