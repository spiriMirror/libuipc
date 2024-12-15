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
    class QueryBuffer
    {
      public:
        auto  view() const noexcept { return m_pairs.view(0, m_size); }
        void  reserve(size_t size) { m_pairs.resize(size); }
        SizeT size() const noexcept { return m_size; }
        auto  viewer() const noexcept { return view().viewer(); }

      private:
        friend class AtomicCountingLBVH;
        SizeT                        m_size = 0;
        muda::DeviceBuffer<Vector2i> m_pairs;
    };

    AtomicCountingLBVH(muda::Stream& stream = muda::Stream::Default()) noexcept;

    void build(muda::CBufferView<LinearBVHAABB> aabbs);

    template <typename Pred>
    void detect(Pred p, QueryBuffer& out_pairs);

    template <typename Pred>
    void query(muda::CBufferView<LinearBVHAABB> query_aabbs, Pred p, QueryBuffer& out_pairs);

  private:
    muda::CBufferView<LinearBVHAABB> m_aabbs;
    muda::DeviceVar<IndexT>          m_cp_num;
    LinearBVH                        m_lbvh;
    Float                            m_reserve_ratio = 1.1;
    muda::Stream&                    m_stream;
};
}  // namespace uipc::backend::cuda

#include "details/atomic_counting_lbvh.inl"