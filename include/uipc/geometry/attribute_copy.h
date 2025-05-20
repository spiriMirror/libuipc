#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/common/type_define.h>
#include <uipc/common/span.h>

namespace uipc::geometry
{
class UIPC_CORE_API AttributeCopy
{
    template <typename T>
    friend class Attribute;

  public:
    enum CopyType
    {
        None = 0,
        SameDim,
        Range,
        Pull,
        Push,
        Pair
    };

    CopyType type() const noexcept;

    /**
     * @brief Dst[i] = Src[i]
     */
    AttributeCopy() noexcept;

    /**
     * @brief Dst[dst_offset + i] = Src[src_offset + i] $$ i \in [0, count) $$
     */
    static AttributeCopy range(SizeT dst_offset, SizeT src_offset, SizeT count) noexcept;
    /**
     * @brief Dst[i] = Src[Mapping[i]] 
     */
    static AttributeCopy pull(span<const SizeT> mapping) noexcept;
    /**
     * @brief Dst[Mapping[i]] = Src[i]
     */
    static AttributeCopy push(span<const SizeT> mapping) noexcept;

    /**
     * @brief Dst[Pairs[i].first] = Src[Pairs[i].second]
     */
    static AttributeCopy pair(span<const std::pair<SizeT, SizeT>> pairs) noexcept;

    /**
     * @brief Dst[i] = Src[i]
     */
    static AttributeCopy same_dim() noexcept;

  private:
    CopyType m_type = CopyType::None;

    // Range
    SizeT m_dst_offset = ~0ull;
    SizeT m_src_offset = ~0ull;
    SizeT m_count      = ~0ull;

    // pull, push
    span<const SizeT> m_mapping;

    // pair mapping
    span<const std::pair<SizeT, SizeT>> m_pairs;

    template <typename T>
    void copy(span<T> dst, span<const T> src) const noexcept;
};
}  // namespace uipc::geometry

#include "details/attribute_copy.inl"