#include <points_from_volume.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/builtin/attribute_name.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/MeshToVolume.h>

namespace uipc::vdb
{
using namespace uipc::geometry;

openvdb::DoubleGrid::Ptr make_grid(const std::vector<openvdb::Vec3s>& points,
                                   const std::vector<openvdb::Vec3I>& triangles,
                                   Float voxel_size = 0.01)
{
    using namespace openvdb;

    math::Transform::Ptr xform = math::Transform::createLinearTransform(voxel_size);

    return tools::meshToSignedDistanceField<DoubleGrid>(
        *xform,
        points,
        triangles,
        std::vector<Vec4I>{},
        /*exteriorBandWidth=*/0.0,
        /*interiorBandWidth=*/std::numeric_limits<Float>::max());
}

SimplicialComplex mesh_to_point_cloud(const SimplicialComplex& sc, Float resolution)
{
    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> triangles;
    points.resize(sc.vertices().size());
    triangles.resize(sc.triangles().size());

    auto V_span = sc.positions().view();
    auto F_span = sc.triangles().topo().view();

    for(auto [i, value] : enumerate(V_span))
        points[i] = openvdb::Vec3s(value.x(), value.y(), value.z());

    for(auto [i, value] : enumerate(F_span))
        triangles[i] = openvdb::Vec3I(value.x(), value.y(), value.z());

    auto grid = make_grid(points, triangles, resolution);

    // copy all voxels to a vector
    SimplicialComplex R;
    auto voxel_size_attr      = R.meta().create<Float>("voxel_size", 0.0);
    view(*voxel_size_attr)[0] = resolution;

    auto pos = R.vertices().create<Vector3>(builtin::position);
    auto sd  = R.vertices().create<Float>("signed_distance");


    R.vertices().resize(grid->activeVoxelCount());
    auto  pos_view = view(*pos);
    auto  sd_view  = view(*sd);
    SizeT idx      = 0;
    for(auto iter = grid->cbeginValueOn(); iter; ++iter)
    {
        if(*iter <= 0.0)
        {
            auto ijk      = iter.getCoord();
            auto xyz      = grid->indexToWorld(ijk);
            sd_view[idx]  = *iter;
            pos_view[idx] = Vector3(xyz.x(), xyz.y(), xyz.z());
            idx++;
        }
    }
    R.vertices().resize(idx);  // resize to actual number of points
    return R;
}
}  // namespace uipc::vdb