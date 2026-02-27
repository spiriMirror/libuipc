#pragma once
#include <muda/ext/spatial_hash/sparse_spatial_hash_impl.h>

namespace muda::spatial_hash
{
class DefaultPredication
{
  public:
    __device__ bool operator()(int i, int j) { return true; }
};

template <typename Hash = Morton<uint32_t>>
class SparseSpatialHash
{
    // algorithm comes from:
    // https://developer.nvidia.com/gpugems/gpugems3/part-v-physics-simulation/chapter-32-broad-phase-collision-detection-cuda
  private:
    using Impl = details::SparseSpatialHashImpl<Hash>;

    Impl m_impl;

  public:
    SparseSpatialHash(muda::Stream& stream = muda::Stream::Default())
        : m_impl(stream)
    {
    }

    /**
     * \brief Detect collision pairs from bounding spheres. Note that:
     * - The collision pairs are unique but not sorted.
     * - All `(i,j)` pairs in collisionPairs satisfy `i < j`.
     * 
     * \param[in] spheres bounding spheres
     * \param[out] collisionPairs output collision pairs
     * \param[in] pred predication function. f: `__device__ (int i, int j) -> bool`. 
     * If `pred(i,j)` is true, then the collision pair `(i,j)` is recorded, 
     * else the collision pair is discarded.
     * \sa \ref DefaultPredication
     */
    template <typename Pred = DefaultPredication>
    void detect(CBufferView<BoundingSphere>  spheres,
                DeviceBuffer<CollisionPair>& collisionPairs,
                Pred&&                       pred = {})
    {
        m_impl.level = 0;
        m_impl.detect(spheres, false, collisionPairs, std::forward<Pred>(pred));
    }

    /**
     * \brief Detect collision pairs from bounding spheres at a specific level (level >= 0).
     * This is used for hierarchical spatial hashing collision detection. Its user's responsibility
     * to set `level` value to a bounding sphere. Normally, higher level means larger bounding sphere.
     * Users should decide a proper range of radius for each level, like:
     * \code{.cpp}
     * BoundingSphere s{o,r};
     * if (r < r0) s.level = 0;
     * else if (r < r1) s.level = 1;
     * else s.level = 2;
     * ...
     * \endcode
     * Note that:
     * - For a level `L`, any bounding spheres has a `level <= L` will be taken into account. 
     * - The collision pairs are unique but not sorted.
     * - All `(i,j)` pairs in collisionPairs satisfy `i < j`.
     * - The `collisionPairs` will be appended with new collision pairs (not cleared).
     * 
     * Typical usage:
     * \code{.cpp}
     *  DeviceBuffer<BoundingSphere> spheres;
     *  
     *  // Your code to fill spheres with proper level
     *  // ...
     * 
     *  DeviceBuffer<CollisionPair> collisionPairs;
     *  
     *  for (int level = 0; level < level_count; ++level)
     *  {
     *      detect(level, spheres, collisionPairs);
     *  }
     * \endcode
     * 
     * \param[in] level the hierarchy level we want to detect
     * \param[in] spheres bounding spheres
     * \param[out] collisionPairs appended output collision pairs
     * \param[in] pred predication function. f: `__device__ (int i, int j) -> bool`. 
     * If `pred(i,j)` is true, then the collision pair `(i,j)` is recorded, 
     * else the collision pair is discarded.
     * 
     * \sa \ref DefaultPredication
     */
    template <typename Pred = DefaultPredication>
    void detect(int                          level,
                CBufferView<BoundingSphere>  spheres,
                DeviceBuffer<CollisionPair>& collisionPairs,
                Pred&&                       pred = {})
    {
        MUDA_KERNEL_ASSERT(level >= 0, "invalid level");
        m_impl.level = level;
        m_impl.detect(spheres, true, collisionPairs, std::forward<Pred>(pred));
    }
};
}  // namespace muda::spatial_hash
