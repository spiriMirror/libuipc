#include <uipc/common/log.h>
#include <uipc/common/enumerate.h>
namespace uipc::geometry
{
template <typename T>
void AttributeCopy::copy(span<T> dst, span<const T> src) const noexcept
{
    switch(m_type)
    {
        case uipc::geometry::AttributeCopy::None: {
            UIPC_WARN_WITH_LOCATION("Attribute copy type is None");
        }
        break;
        case uipc::geometry::AttributeCopy::SameDim: {
            UIPC_ASSERT(src.size() == dst.size(),
                        "Attribute size mismatch, dst is {}, src is {}.",
                        dst.size(),
                        src.size());
            std::ranges::copy(src, dst.begin());
        }
        break;
        case uipc::geometry::AttributeCopy::Range: {
            auto src_subspan = src.subspan(m_src_offset, m_count);
            auto dst_subspan = dst.subspan(m_dst_offset, m_count);
            std::ranges::copy(src_subspan, dst_subspan.begin());
        }
        break;
        case uipc::geometry::AttributeCopy::Pull: {
            auto pull_mapping = m_mapping;
            UIPC_ASSERT(pull_mapping.size() == dst.size(),
                        "Pull mapping size mismatch, dst size is {}, mapper size is {}",
                        dst.size(),
                        pull_mapping.size());
            for(auto [i, j] : enumerate(pull_mapping))
            {
                dst[i] = src[j];
            }
        }
        break;
        case uipc::geometry::AttributeCopy::Push: {
            auto push_mapping = m_mapping;
            UIPC_ASSERT(push_mapping.size() == src.size(),
                        "Push mapping size mismatch, src size is {}, mapper size is {}",
                        src.size(),
                        push_mapping.size());
            for(auto [i, j] : enumerate(push_mapping))
            {
                dst[j] = src[i];
            }
        }
        break;
        case uipc::geometry::AttributeCopy::Pair: {
            for(auto [i, j] : m_pairs)
            {
                dst[i] = src[j];
            }
        }
        break;
        default:
            UIPC_ASSERT(false, "Invalid attribute copy type");
            break;
    }
}
}  // namespace uipc::geometry
