#include <muda/cub/device/device_reduce.h>
#include <muda/cub/device/device_radix_sort.h>
#include <muda/cub/device/device_run_length_encode.h>
#include <muda/cub/device/device_scan.h>
#include <muda/cub/device/device_select.h>

namespace muda::spatial_hash::details
{
template <typename Hash>
inline void SparseSpatialHashImpl<Hash>::setup_hash_table()
{
    calculate_hash_table_basic_info();
    fill_hash_cells();
    count_object_per_cell();
}

template <typename Hash>
template <typename Pred>
inline void SparseSpatialHashImpl<Hash>::detect(CBufferView<BoundingSphere> boundingSphereList,
                                                bool append,
                                                DeviceBuffer<CollisionPair>& collisionPairs,
                                                Pred&& pred)
{
    spheres = boundingSphereList;
    setup_hash_table();
    balanced_setup_collision_pairs(append, collisionPairs, std::forward<Pred>(pred));
}

template <typename Hash>
void SparseSpatialHashImpl<Hash>::calculate_hash_table_basic_info()
{
    auto count = spheres.size();

    BufferLaunch(m_stream)
        .clear(allRadius)
        .resize(allRadius, count)
        .clear(allCoords)
        .resize(allCoords, count);

    constexpr auto float_max = std::numeric_limits<float>::max();
    const Vector3  vector3_max(float_max, float_max, float_max);

    ParallelFor(0, m_stream)  //
        .apply(count,
               [spheres   = spheres.cviewer().name("spheres"),
                allRadius = allRadius.viewer().name("allRadius"),
                allCoords = allCoords.viewer().name("allCoords"),
                vector3_max,
                level = this->level] __device__(int i) mutable
               {
                   const auto& s = spheres(i);
                   allRadius(i)  = s.level == level ? s.r : 0.0f;
                   allCoords(i)  = s.level <= level ? s.o : vector3_max;
               });

    DeviceReduce().Max(allRadius.data(), maxRadius.data(), count);


    DeviceReduce().Reduce(
        allCoords.data(),
        minCoord.data(),
        count,
        [] __host__ __device__(const Vector3& a, const Vector3& b) -> Vector3
        { return a.cwiseMin(b); },
        vector3_max);

    // Scaling the bounding sphere of each object by sqrt(2) [we will use proxy spheres]
    // and ensuring that the grid cell is at least 1.5 times
    // as large as the scaled bounding sphere of the largest object.
    //https://developer.nvidia.com/gpugems/gpugems3/part-v-physics-simulation/chapter-32-broad-phase-collision-detection-cuda

    float maxRadius = this->maxRadius;
    empty_level     = maxRadius == 0.0f;

    if(empty_level)  // no object in this level
        return;

    auto scaledCellSize           = maxRadius * 2 * 1.5 * 1.5;
    h_spatialHashConfig.coord_min = minCoord;
    // shift the coord_min by the scaledMaxRadius, which is much safer than the original maxRadius
    h_spatialHashConfig.coord_min -= scaledCellSize * Vector3::Ones();
    h_spatialHashConfig.cell_size = scaledCellSize;

    // upload
    spatialHashConfig = h_spatialHashConfig;
}

template <typename Hash>
void SparseSpatialHashImpl<Hash>::fill_hash_cells()
{
    if(empty_level)
        return;

    using namespace muda;

    int size  = spheres.size();
    int count = 8 * size + 1;  //

    BufferLaunch(m_stream)
        //.clear(cellArrayValue)
        .resize(cellArrayValue, count)
        //.clear(cellArrayKey)
        .resize(cellArrayKey, count)
        //.clear(cellArrayValueSorted)
        .resize(cellArrayValueSorted, count)
        //.clear(cellArrayKeySorted)
        .resize(cellArrayKeySorted, count)
        //.clear(uniqueKey)
        .resize(uniqueKey, count)
        //.clear(objCountInCell)
        .resize(objCountInCell, count)
        //.clear(objCountInCellPrefixSum)
        .resize(objCountInCellPrefixSum, count)
        //.clear(collisionPairCount)
        .resize(collisionPairCount, count)
        //.clear(collisionPairPrefixSum)
        .resize(collisionPairPrefixSum, count);

    ParallelFor(0, m_stream)  //
        .apply(spheres.size(),
               [spheres           = spheres.cviewer(),
                spatialHashConfig = spatialHashConfig.viewer(),
                cellArrayValue = make_dense_2d(cellArrayValue.data(), size, 8),
                cellArrayKey   = make_dense_2d(cellArrayKey.data(), size, 8),
                level          = this->level] __device__(int i) mutable
               {
                   BoundingSphere s  = spheres(i);
                   auto&          sh = *spatialHashConfig;

                   auto proxySphere = s;
                   // https://developer.nvidia.com/gpugems/gpugems3/part-v-physics-simulation/chapter-32-broad-phase-collision-detection-cuda
                   // Scaling the bounding sphere of each object by sqrt(2), here we take 1.5(>1.414)
                   proxySphere.r *= 1.5;

                   auto     o        = s.o;
                   Vector3u ijk      = sh.cell(o);
                   auto     hash     = sh.hash_cell(ijk);
                   auto     cellSize = sh.cell_size;
                   // print("beginFillHashCells cellSize=%f\n", cellSize);
                   auto objectId = i;


                   Cell homeCell(hash, objectId);

                   // ...[i*8+0][i*8+1][i*8+2][i*8+3][i*8+4][i*8+5][i*8+6][i*8+7]...
                   // ...[hcell][pcell][pcell] ...   [  x  ][  x  ][  x  ][  x  ]...


                   // fill the cell that contains the center of the current sphere
                   homeCell.set_as_home(ijk);
                   // homeCell.ijk = ijk;
                   Vector3 xyz = sh.cell_center_coord(ijk);

                   //find the cloest 7 neighbor cells
                   Vector3i dxyz;
#pragma unroll
                   for(int i = 0; i < 3; ++i)
                       dxyz(i) = o(i) > xyz(i) ? 1 : -1;

                   auto cal_cell = [&](const Vector3i& dxyz) -> Vector3u

                   {
                       Vector3i res = (ijk.cast<I32>() + dxyz);
                       return res.cast<U32>();
                   };

                   Vector3u cells[7] = {cal_cell(Vector3i(dxyz.x(), 0, 0)),
                                        cal_cell(Vector3i(0, dxyz.y(), 0)),
                                        cal_cell(Vector3i(0, 0, dxyz.z())),
                                        cal_cell(Vector3i(0, dxyz.y(), dxyz.z())),
                                        cal_cell(Vector3i(dxyz.x(), 0, dxyz.z())),
                                        cal_cell(Vector3i(dxyz.x(), dxyz.y(), 0)),
                                        cal_cell(dxyz)};

                   // the cell size (3d)
                   Vector3 size(cellSize, cellSize, cellSize);

                   int idx = 1;  //goes from 1 -> 7. idx = 0 is for the homeCell
#pragma unroll
                   for(int i = 0; i < 7; ++i)
                   {
                       Vector3 min = sh.coord(cells[i]);
                       Vector3 max = min + size;
                       AABB    aabb(min, max);

                       // use proxySphere to test
                       // whether the current sphere overlaps the neighbor cell
                       if(intersect(proxySphere, aabb))
                       {
                           homeCell.set_overlap(cells[i]);
                           auto hash = sh.hash_cell(cells[i]);
                           Cell phantom(hash, objectId);
                           phantom.set_as_phantom(ijk, cells[i]);
                           // phantom.ijk            = cells[i];
                           phantom.ctlbit.overlap = homeCell.ctlbit.overlap;
                           cellArrayValue(objectId, idx++) = phantom;
                       }
                   }

                   //set the home cell
                   cellArrayValue(objectId, 0) = homeCell;

                   // turn off othese non-overlap neighbor cells if we do have.
                   for(; idx < 8; ++idx)
                       cellArrayValue(objectId, idx) = Cell(~0, ~0);

                   // fill the key for later sorting
                   for(int i = 0; i < 8; ++i)
                       cellArrayKey(objectId, i) = cellArrayValue(objectId, i).cid;
               });

    Launch(1, 1, 0, m_stream)  //
        .apply(
            [cellArrayValue = cellArrayValue.viewer(),
             cellArrayKey   = cellArrayKey.viewer()] __device__() mutable
            {
                cellArrayKey(cellArrayKey.total_size() - 1)     = -1;
                cellArrayValue(cellArrayValue.total_size() - 1) = Cell(~0, ~0);
            });
}

template <typename Hash>
void SparseSpatialHashImpl<Hash>::count_object_per_cell()
{
    if(empty_level)
        return;

    DeviceRadixSort(m_stream).SortPairs((uint32_t*)cellArrayKey.data(),  //in
                                        (uint32_t*)cellArrayKeySorted.data(),  //out
                                        cellArrayValue.data(),        //in
                                        cellArrayValueSorted.data(),  //out
                                        cellArrayValue.size());

    auto count = uniqueKey.size();
    DeviceRunLengthEncode(m_stream).Encode(cellArrayKeySorted.data(),  // in
                                           uniqueKey.data(),           // out
                                           objCountInCell.data(),      // out
                                           uniqueKeyCount.data(),      // out
                                           count);

    int h_uniqueKeyCount = uniqueKeyCount;

    BufferLaunch(m_stream)
        .resize(uniqueKey, h_uniqueKeyCount)
        .resize(objCountInCell, h_uniqueKeyCount)
        .resize(objCountInCellPrefixSum, h_uniqueKeyCount)
        .resize(collisionPairCount, h_uniqueKeyCount)
        .resize(collisionPairPrefixSum, h_uniqueKeyCount);

    validCellCount = h_uniqueKeyCount - 1;

    // we still prefix sum the uniqueKeyCount cell-object-pair
    // because we need to know the total number of collision pairs
    // which is at the last element of the prefix sum array
    DeviceScan(m_stream).ExclusiveSum(objCountInCell.data(),           // in
                                      objCountInCellPrefixSum.data(),  // out
                                      validCellCount + 1);
}

template <typename Hash>
template <typename Pred>
void SparseSpatialHashImpl<Hash>::simple_setup_collision_pairs(Pred&& pred,
                                                               DeviceBuffer<CollisionPair>& collisionPairs)
{
    if(empty_level)
        return;

    // using a simple in-thread counting way to get the total number of collision pairs
    simple_count_collision_pairs(std::forward<Pred>(pred));
    alloc_collision_pair_list(collisionPairs, sum);
    simple_fill_collision_pair_list(collisionPairs, std::forward<Pred>(pred));
}

template <typename Hash>
template <typename Pred>
void SparseSpatialHashImpl<Hash>::simple_count_collision_pairs(Pred&& pred)
{
    using CallableType = raw_type_t<Pred>;

    using namespace muda;

    muda::ParallelFor(0, m_stream)  //
        .apply(validCellCount,
               [spheres                 = spheres.viewer(),
                objCountInCell          = objCountInCell.viewer(),
                objCountInCellPrefixSum = objCountInCellPrefixSum.viewer(),
                cellArrayValueSorted    = cellArrayValueSorted.viewer(),
                collisionPairCount      = collisionPairCount.viewer(),
                pred                    = std::forward<Pred>(pred),
                level = this->level] __device__(int cell) mutable
               {
                   int size      = objCountInCell(cell);
                   int offset    = objCountInCellPrefixSum(cell);
                   int pairCount = 0;
                   // print("cell %d has %d objects\n", cell, size);
                   for(int i = 0; i < size; ++i)
                   {
                       auto cell0 = cellArrayValueSorted(offset + i);
                       auto oid0  = cell0.oid;
                       auto s0    = spheres(oid0);
                       if(s0.level < level)
                           continue;
                       for(int j = i + 1; j < size; ++j)
                       {
                           auto cell1 = cellArrayValueSorted(offset + j);
                           auto oid1  = cell1.oid;
                           auto s1    = spheres(oid1);
                           // print("test => %d,%d\n", oid0, oid1);
                           if(!Cell::allow_ignore(cell0, cell1) && intersect(s0, s1)
                              && pred(oid0, oid1))  // user predicate
                           {
                               //print("pair");
                               ++pairCount;
                           }
                       }
                   }
                   collisionPairCount(cell) = pairCount;
               })
        .wait();

    int keycount = uniqueKeyCount;
    if(keycount)
    {
        DeviceScan(m_stream).ExclusiveSum(
            collisionPairCount.data(), collisionPairPrefixSum.data(), keycount);
        auto lastOffset = validCellCount;

        BufferLaunch(m_stream)  //
            .copy(&sum, std::as_const(collisionPairPrefixSum).view(lastOffset))
            .wait();
    }
    else
        sum = 0;
}

template <typename Hash>
void SparseSpatialHashImpl<Hash>::alloc_collision_pair_list(DeviceBuffer<CollisionPair>& collisionPairs,
                                                            int totalCollisionPairCount)
{
    pairListOffset = collisionPairs.size();
    collisionPairs.resize(collisionPairs.size() + totalCollisionPairCount);
}

template <typename Hash>
template <typename Pred>
void SparseSpatialHashImpl<Hash>::simple_fill_collision_pair_list(DeviceBuffer<CollisionPair>& collisionPairs,
                                                                  Pred&& pred)
{
    using namespace muda;

    muda::ParallelFor(0, m_stream)
        .apply(validCellCount,
               [spheres                 = spheres.viewer(),
                objCountInCell          = objCountInCell.viewer(),
                objCountInCellPrefixSum = objCountInCellPrefixSum.viewer(),
                cellArrayValueSorted    = cellArrayValueSorted.viewer(),
                collisionPairCount      = collisionPairCount.viewer(),
                collisionPairPrefixSum  = collisionPairPrefixSum.viewer(),
                collisionPairs = collisionPairs.view(pairListOffset).viewer(),
                pred           = std::forward<Pred>(pred),
                level          = this->level] __device__(int cell) mutable
               {
                   int size       = objCountInCell(cell);
                   int offset     = objCountInCellPrefixSum(cell);
                   int pairOffset = collisionPairPrefixSum(cell);
                   int index      = 0;
                   for(int i = 0; i < size; ++i)
                   {
                       auto cell0 = cellArrayValueSorted(offset + i);
                       auto oid0  = cell0.oid;
                       auto s0    = spheres(oid0);

                       if(s0.level < level)
                           continue;

                       for(int j = i + 1; j < size; ++j)
                       {
                           auto cell1 = cellArrayValueSorted(offset + j);
                           auto oid1  = cell1.oid;
                           auto s1    = spheres(oid1);
                           //print("test => %d,%d\n", oid0, oid1);
                           if(!Cell::allow_ignore(cell0, cell1)  // cell0, cell1 are created by test the proxy sphere
                              && intersect(s0, s1)  // test the bounding spheres to get exact collision result
                              && pred(oid0, oid1))  // user predicate
                           {
                               CollisionPair p{oid0, oid1};
                               collisionPairs(pairOffset + index) = p;
                               ++index;
                           }
                       }
                   }
               });
}

constexpr int ij_to_cell_local_index(int i, int j, int objCount)
{
    return (objCount - 1 + objCount - i) * i / 2 + j - i - 1;
}

template <typename Hash>
template <typename Pred>
void SparseSpatialHashImpl<Hash>::balanced_setup_collision_pairs(
    bool append, DeviceBuffer<CollisionPair>& collisionPairs, Pred&& pred)
{
    if(empty_level)
        return;

    cellToCollisionPairUpperBound.resize(validCellCount + 1);
    cellToCollisionPairUpperBoundPrefixSum.resize(validCellCount + 1);

    // eg:
    // objCountInCell                           = [1, 2, 3]
    // cellToCollisionPairUpperBound           = [1, 4, 9, x]
    ParallelFor(0, m_stream)
        .kernel_name("setup_collision_pairs_upper_bound")
        .apply(validCellCount,
               [objCountInCell = objCountInCell.viewer(),
                cellToCollisionPairUpperBound =
                    cellToCollisionPairUpperBound.view(0, validCellCount).viewer()] __device__(int cell) mutable
               {
                   int size = objCountInCell(cell);
                   cellToCollisionPairUpperBound(cell) = size * (size - 1) / 2 + 1;  // +1 for sentinel
               });

    // e.g.
    // cellToCollisionPairUpperBound           = [1, 4, 9, x]
    // cellToCollisionPairUpperBoundPrefixSum  = [0, 1, 5, 14]
    DeviceScan(m_stream).ExclusiveSum(cellToCollisionPairUpperBound.data(),
                                      cellToCollisionPairUpperBoundPrefixSum.data(),
                                      validCellCount + 1);

    // e.g.
    // count = 14
    int collisionPairCountUpperBound = 0;
    BufferLaunch(m_stream).copy<int>(&collisionPairCountUpperBound,
                                     cellToCollisionPairUpperBoundPrefixSum.view(validCellCount));

    // e.g.
    //                                              0  1  2  3  4  5  6  7  8  9 10 11 12 13
    // potentialCollisionPairIdToCellIndexBuffer = [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1]
    BufferLaunch(m_stream)
        .resize(potentialCollisionPairIdToCellIndexBuffer, collisionPairCountUpperBound)
        .fill(potentialCollisionPairIdToCellIndexBuffer.view(), 0);
    ParallelFor(0, m_stream)
        .kernel_name("fill_last_collision_pair_count")
        .apply(validCellCount,
               [objCountInCell = objCountInCell.cviewer(),
                cellToCollisionPairUpperBound =
                    cellToCollisionPairUpperBound.view(0, validCellCount).viewer(),
                cellToCollisionPairUpperBoundPrefixSum =
                    cellToCollisionPairUpperBoundPrefixSum.cviewer(),
                potentialCollisionPairIdToCellIndexBuffer =
                    potentialCollisionPairIdToCellIndexBuffer
                        .view(0, collisionPairCountUpperBound)
                        .viewer()] __device__(int cell) mutable
               {
                   int size = objCountInCell(cell);
                   MUDA_KERNEL_ASSERT(size > 0,
                                      "Fatal Algo Error: objCountInCell(%d)=%d, an empty cell shouldn't be recorded, something goes wrong!",
                                      cell,
                                      size);
                   int start = cellToCollisionPairUpperBoundPrefixSum(cell);
                   int upper_bound = cellToCollisionPairUpperBound(cell);
                   potentialCollisionPairIdToCellIndexBuffer(start + upper_bound - 1) = 1;
               });

    BufferLaunch(m_stream).resize(potentialCollisionPairIdToCellIndex,
                                  collisionPairCountUpperBound);

    DeviceScan(m_stream).ExclusiveSum(potentialCollisionPairIdToCellIndexBuffer.data(),
                                      potentialCollisionPairIdToCellIndex.data(),
                                      collisionPairCountUpperBound);

    if(!append)
        BufferLaunch(m_stream).clear(collisionPairs);

    auto collisionPairOffset = collisionPairs.size();

    BufferLaunch(m_stream)
        .resize(collisionPairBuffer, collisionPairCountUpperBound)
        .resize(collisionPairs, collisionPairOffset + collisionPairCountUpperBound);


    // e.g.
    // cellArrayValueSorted                         = [0, 1, 2, 3, 4, 5]
    // objCountInCell                               = [1, 2, 3]
    // cellToCollisionPairUpperBoundPrefixSum       = [0, 1, 5, 14]
    //
    // potentialCollisionPairIdToCellIndex = [0, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3, 3]
    ParallelFor(0, m_stream)
        .kernel_name("fill_collision_pairs")
        .apply(
            collisionPairCountUpperBound,
            [spheres        = spheres.cviewer().name("spheres"),
             objCountInCell = objCountInCell.cviewer().name("objCountInCell"),
             cellOffsets = objCountInCellPrefixSum.cviewer().name("cellOffsets"),
             cellToCollisionPairUpperBound =
                 cellToCollisionPairUpperBound.cviewer().name("cellToCollisionPairUpperBound"),
             cellArrayValueSorted = cellArrayValueSorted.cviewer().name("cellArrayValueSorted"),
             cellToCollisionPairUpperBoundPrefixSum =
                 cellToCollisionPairUpperBoundPrefixSum.cviewer().name("cellToCollisionPairUpperBoundPrefixSum"),
             potentialCollisionPairIdToCellIndex =
                 potentialCollisionPairIdToCellIndex.cviewer().name("potentialCollisionPairIdToCellIndex"),
             potentialCollisionPairIdToCellIndexBuffer =
                 potentialCollisionPairIdToCellIndexBuffer.cviewer().name("potentialCollisionPairIdToCellIndexBuffer"),
             collisionPairBuffer = collisionPairBuffer.viewer().name("collisionPairBuffer"),
             pred  = std::forward<Pred>(pred),
             level = this->level] __device__(int cpI) mutable
            {
                int cellIndex = potentialCollisionPairIdToCellIndex(cpI);

                // int upper_bound = cellToCollisionPairUpperBound(cellIndex);  // check the index is valid

                int start = cellToCollisionPairUpperBoundPrefixSum(cellIndex);

                int objCount = objCountInCell(cellIndex);

                int cellOffset = cellOffsets(cellIndex);

                int cellLocalIndex = cpI - start;

                collisionPairBuffer(cpI) = CollisionPair::invalid();

                if(cellLocalIndex == 0)  // ignore the first sentinel
                    return;

                cellLocalIndex -= 1;

                // use the formula to get the i, j

                int i = objCount - 2
                        - floor(sqrt(-8.0 * cellLocalIndex + 4 * objCount * (objCount - 1) - 7) / 2.0
                                - 0.5);
                int j = cellLocalIndex + i + 1 - objCount * (objCount - 1) / 2
                        + (objCount - i) * ((objCount - i) - 1) / 2;

                // printf("CellLocalId=%d, i=%d, j=%d,objCount=%d\n", cellLocalIndex, i, j, objCount);

                MUDA_KERNEL_ASSERT(
                    i >= 0 && j >= 0 && i < objCount && j < objCount, "i=%d, j=%d", i, j);

                MUDA_KERNEL_ASSERT(ij_to_cell_local_index(i, j, objCount) == cellLocalIndex,
                                   "numerical error happen!"
                                   "i=%d, j=%d, objCount=%d, cellLocalIndex=%d",
                                   i,
                                   j,
                                   objCount,
                                   cellLocalIndex);

                Cell cell0 = cellArrayValueSorted(cellOffset + i);
                Cell cell1 = cellArrayValueSorted(cellOffset + j);

                int oid0 = cell0.oid;
                int oid1 = cell1.oid;

                // print("cellOffset=%d, ij=(%d,%d), oid=(%d,%d)\n", cellOffset, i, j, oid0, oid1);

                auto s0 = spheres(oid0);
                auto s1 = spheres(oid1);

                if(s0.level > level || s1.level > level)
                    return;

                if(s0.level < level && s1.level < level)
                    return;

                if(!Cell::allow_ignore(cell0, cell1)  // cell0, cell1 are created by test the proxy sphere
                   && intersect(s0, s1)  // test the bounding spheres to get exact collision result
                   && pred(oid0, oid1))  // user predicate
                {
                    collisionPairBuffer(cpI) = CollisionPair{oid0, oid1};
                }
            });

    // select the valid collision pairs
    DeviceSelect(m_stream).If(collisionPairBuffer.data(),
                              collisionPairs.view(collisionPairOffset).data(),
                              validCollisionPairCount.data(),
                              collisionPairCountUpperBound,
                              [] CUB_RUNTIME_FUNCTION(const CollisionPair& p) -> bool
                              { return p.is_valid(); });

    BufferLaunch(m_stream).resize(collisionPairs, collisionPairOffset + validCollisionPairCount);
}
}  // namespace muda::spatial_hash::details