#pragma once
#include <uipc/common/span.h>
#include <collision_detection/linear_bvh.h>
#include <muda/buffer/device_buffer.h>
#include <muda/launch.h>

namespace uipc::backend::cuda
{
class AtomicCountingLBVH
{
  public:
    AtomicCountingLBVH(muda::Stream& stream = muda::Stream::Default()) noexcept;
    void reserve(size_t size);

    void build(muda::CBufferView<LinearBVHAABB> aabbs);

    template <typename Pred>
    muda::BufferView<Vector2i> detect(Pred p);

    template <typename Pred>
    muda::BufferView<Vector2i> query(muda::CBufferView<LinearBVHAABB> query_aabbs, Pred p);

    /**
     * @param query_obj A callable objects, e.g. indices of surface vertices, edges, etc.
     * @param get_aabb  f: AABB (const T& obj)
     */
    template <typename GetQueryAABB, typename ObjectT, typename Pred>
    muda::BufferView<Vector2i> query(muda::CBufferView<ObjectT> query_obj,
                                     GetQueryAABB               get_aabb,
                                     Pred                       p);


  private:
    muda::CBufferView<LinearBVHAABB> m_aabbs;
    muda::DeviceVar<IndexT>          m_cp_num;
    LinearBVH                        m_lbvh;
    muda::DeviceBuffer<Vector2i>     m_pairs;
    Float                            m_reserve_ratio = 1.1;
    muda::Stream&                    m_stream;
};
}  // namespace uipc::backend::cuda

#include "details/atomic_counting_lbvh.inl"