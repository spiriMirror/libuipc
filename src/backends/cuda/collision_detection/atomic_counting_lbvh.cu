#include <collision_detection/atomic_counting_lbvh.h>

namespace uipc::backend::cuda
{
AtomicCountingLBVH::AtomicCountingLBVH(muda::Stream& stream) noexcept
    : m_stream(stream)
{
}

void AtomicCountingLBVH::build(muda::CBufferView<LinearBVHAABB> aabbs)
{
    m_aabbs = aabbs;
    m_lbvh.build(aabbs);
}
}  // namespace uipc::backend::cuda
