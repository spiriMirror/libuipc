#include <uipc/geometry/attribute_copy.h>

namespace uipc::geometry
{
AttributeCopy::AttributeCopy() noexcept
    : m_type{CopyType::SameDim}
{
}

auto AttributeCopy::type() const noexcept -> CopyType
{
    return m_type;
}

AttributeCopy AttributeCopy::range(SizeT dst_offset, SizeT src_offset, SizeT count) noexcept
{
    AttributeCopy copy;
    copy.m_type       = CopyType::Range;
    copy.m_dst_offset = dst_offset;
    copy.m_src_offset = src_offset;
    copy.m_count      = count;
    return copy;
}
AttributeCopy AttributeCopy::pull(span<const SizeT> mapping) noexcept
{
    AttributeCopy copy;
    copy.m_type    = CopyType::Pull;
    copy.m_mapping = mapping;
    return copy;
}
AttributeCopy AttributeCopy::push(span<const SizeT> mapping) noexcept
{
    AttributeCopy copy;
    copy.m_type    = CopyType::Push;
    copy.m_mapping = mapping;
    return copy;
}
AttributeCopy AttributeCopy::pair(span<const std::pair<SizeT, SizeT>> pairs) noexcept
{
    AttributeCopy copy;
    copy.m_type  = CopyType::Pair;
    copy.m_pairs = pairs;
    return copy;
}
AttributeCopy AttributeCopy::same_dim() noexcept
{
    AttributeCopy copy;
    copy.m_type = CopyType::SameDim;
    return copy;
}
}  // namespace uipc::geometry
