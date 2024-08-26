#pragma once
#include <muda/muda_def.h>
#include <muda/tools/debug_log.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>
#include <Eigen/Geometry>

namespace uipc::backend::cuda::distance
{
enum class PointPointDistanceType : unsigned char
{
    PP = 0,  // point-point, the shortest distance is the distance between the two points
};

enum class PointEdgeDistanceType : unsigned char
{
    PP_PE0 = 0,  // point-edge, the shortest distance is the distance between the point and the point 0 in edge
    PP_PE1 = 1,  // point-edge, the shortest distance is the distance between the point and the point 1 in edge
    PE = 2,  // point-edge, the shortest distance is the distance between the point and some point on the edge
};

enum class PointTriangleDistanceType : unsigned char
{
    PP_PT0 = 0,  // point-triangle, the closest point is the point 0 in triangle
    PP_PT1 = 1,  // point-triangle, the closest point is the point 1 in triangle
    PP_PT2 = 2,  // point-triangle, the closest point is the point 2 in triangle
    PE_PT0T1 = 3,  // point-triangle, the closest point is on the edge (t0, t1)
    PE_PT1T2 = 4,  // point-triangle, the closest point is on the edge (t1, t2)
    PE_PT2T0 = 5,  // point-triangle, the closest point is on the edge (t2, t0)
    PT       = 6,  // point-triangle, the closest point is on the triangle
};

enum class EdgeEdgeDistanceType : unsigned char
{
    PP_Ea0Eb0 = 0,  // point-point, the shortest distance is the distance between the point 0 in edge a and the point 0 in edge b
    PP_Ea0Eb1 = 1,  // point-point, the shortest distance is the distance between the point 0 in edge a and the point 1 in edge b
    PE_Ea0Eb0Eb1 = 2,  // point-edge, the shortest distance is the distance between the point 0 in edge a and some point the edge b
    PP_Ea1Eb0 = 3,  // point-point, the shortest distance is the distance between the point 1 in edge a and the point 0 in edge b
    PP_Ea1Eb1 = 4,  // point-point, the shortest distance is the distance between the point 1 in edge a and the point 1 in edge b
    PE_Ea1Eb0Eb1 = 5,  // point-edge, the shortest distance is the distance between the point 1 in edge a and some point the edge b
    PE_Eb0Ea0Ea1 = 6,  // point-edge, the shortest distance is the distance between the point 0 in edge b and some point the edge a
    PE_Eb1Ea0Ea1 = 7,  // point-edge, the shortest distance is the distance between the point 1 in edge b and some point the edge a
    EE = 8,  // edge-edge, the shortest distance is the distance between some point on edge a and some point on edge b
};


template <class T, int dim>
MUDA_GENERIC PointPointDistanceType point_point_distance_type(
    const Eigen::Vector<T, dim>& p0, const Eigen::Vector<T, dim>& p1);

template <class T, int dim>
MUDA_GENERIC PointEdgeDistanceType
point_edge_distance_type(const Eigen::Vector<T, dim>& p,
                         const Eigen::Vector<T, dim>& e0,
                         const Eigen::Vector<T, dim>& e1);

template <class T, int dim>
MUDA_GENERIC PointEdgeDistanceType
point_edge_distance_type(const Eigen::Vector<T, dim>& p,
                         const Eigen::Vector<T, dim>& e0,
                         const Eigen::Vector<T, dim>& e1,
                         T&                           ratio);

template <class T>
MUDA_GENERIC PointTriangleDistanceType
point_triangle_distance_type(const Eigen::Vector<T, 3>& p,
                             const Eigen::Vector<T, 3>& t0,
                             const Eigen::Vector<T, 3>& t1,
                             const Eigen::Vector<T, 3>& t2);

// a more robust implementation of http://geomalgorithms.com/a07-_distance.html
template <class T>
MUDA_GENERIC EdgeEdgeDistanceType edge_edge_distance_type(const Eigen::Vector<T, 3>& ea0,
                                                          const Eigen::Vector<T, 3>& ea1,
                                                          const Eigen::Vector<T, 3>& eb0,
                                                          const Eigen::Vector<T, 3>& eb1);

}  // namespace muda::distance

#include "details/distance_type.inl"
