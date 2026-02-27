#pragma once
#include <muda/ext/eigen/eigen_core_cxx20.h>
#include <Eigen/Geometry>
#include <muda/muda_def.h>

namespace muda::spatial_hash
{
class BoundingSphere
{
    using Vector3 = Eigen::Vector3f;

  public:
    MUDA_GENERIC BoundingSphere(const Vector3& o, float r)
        : o(o)
        , r(r)
    {
    }
    MUDA_GENERIC BoundingSphere() = default;

    Vector3 o     = Vector3::Zero();
    float   r     = 0.0f;
    int     level = 0;
};

class AABB
{
    using Vector3 = Eigen::Vector3f;

  public:
    Vector3 max;
    Vector3 min;

    MUDA_GENERIC AABB(const Vector3& min, const Vector3& max)
        : min(min)
        , max(max)
    {
    }

    MUDA_GENERIC AABB(const AABB& l, const AABB& r)
    {
        max = l.max.cwiseMax(r.max);
        min = l.min.cwiseMin(r.min);
    }

    MUDA_GENERIC Vector3 center() const { return (max + min) / 2; }

    MUDA_GENERIC float radius() const { return (max - min).norm() / 2; }
};

MUDA_INLINE MUDA_GENERIC float squared_distance(const Eigen::Vector3f& p, AABB b)
{
    float sq_dist = 0.0f;
#pragma unroll
    for(int i = 0; i < 3; i++)
    {
        // for each axis count any excess distance outside box extents
        float v = p[i];
        if(v < b.min[i])
            sq_dist += (b.min[i] - v) * (b.min[i] - v);
        if(v > b.max[i])
            sq_dist += (v - b.max[i]) * (v - b.max[i]);
    }
    return sq_dist;
}

MUDA_INLINE MUDA_GENERIC float distance(const Eigen::Vector3f& p, AABB b)
{
    return ::sqrt(squared_distance(p, b));
}

MUDA_INLINE MUDA_GENERIC bool intersect(const BoundingSphere& s, const AABB& b)
{
    // Compute squared distance between sphere center and AABB
    // the sqrt(dist) is fine to use as well, but this is faster.
    float sqDist = squared_distance(s.o, b);

    // Sphere and AABB intersect if the (squared) distance between them is
    // less than the (squared) sphere radius.
    return sqDist <= s.r * s.r;
}

MUDA_INLINE MUDA_GENERIC bool intersect(const BoundingSphere& lhs, const BoundingSphere& rhs)
{
    float r = lhs.r + rhs.r;
    return (lhs.o - rhs.o).squaredNorm() <= r * r;
}

MUDA_INLINE MUDA_GENERIC bool intersect(const AABB& l, const AABB& r)
{
    Eigen::Vector3i c;
#pragma unroll
    for(int i = 0; i < 3; ++i)
        c[i] = l.min[i] <= r.max[i] && l.max[i] >= r.min[i];
    return c.all();
}
}  // namespace muda::spatial_hash
