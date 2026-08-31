#pragma once
#include <uipc/common/span.h>
#include <collision_detection/linear_bvh.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
class AtomicCountingLBVH
{
  public:
    class QueryBuffer
    {
      public:
        auto view() const noexcept { return m_pairs.view(0, m_size); }
        void reserve(size_t size)
        {
            m_pairs.reserve_discard(size);
            m_pairs.resize_discard(size);
        }
        SizeT size() const noexcept { return m_size; }
        auto  viewer() const noexcept { return view().viewer(); }

      private:
        friend class AtomicCountingLBVH;
        SizeT                             m_size = 0;
        cuda_tool::DeviceBuffer<Vector2i> m_pairs;
    };

    AtomicCountingLBVH(cuda_tool::Stream& stream = cuda_tool::Stream::Default()) noexcept;

    void build(cuda_tool::CBufferView<LinearBVHAABB> aabbs);

    template <typename Pred>
    void detect(Pred p, QueryBuffer& out_pairs);

    template <typename Pred>
    void query(cuda_tool::CBufferView<LinearBVHAABB> query_aabbs, Pred p, QueryBuffer& out_pairs);

  private:
    cuda_tool::CBufferView<LinearBVHAABB> m_aabbs;
    cuda_tool::DeviceVar<IndexT>          m_cp_num;
    LinearBVH                             m_lbvh;
    Float                                 m_reserve_ratio = 1.1;
    cuda_tool::Stream&                    m_stream;
};
}  // namespace uipc::backend::cuda

#include "details/atomic_counting_lbvh.inl"
