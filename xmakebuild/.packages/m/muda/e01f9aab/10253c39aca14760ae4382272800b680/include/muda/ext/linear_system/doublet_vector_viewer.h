#pragma once

#include <string>
#include <muda/viewer/viewer_base.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>

/*
* - 2024/2/23 remove viewer's subview, view's subview is enough
*/

namespace muda
{
template <bool IsConst, typename T, int N>
class DoubletVectorViewerT : public ViewerBase<IsConst>
{
    using Base = ViewerBase<IsConst>;
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    template <bool OtherIsConst, typename U, int M>
    friend class DoubletVectorViewerT;

    MUDA_VIEWER_COMMON_NAME(DoubletVectorViewerT);

  public:
    using ValueT      = std::conditional_t<N == 1, T, Eigen::Matrix<T, N, 1>>;
    using ConstViewer = DoubletVectorViewerT<true, T, N>;
    using NonConstViewer = DoubletVectorViewerT<false, T, N>;
    using ThisViewer     = DoubletVectorViewerT<IsConst, T, N>;

    struct CDoublet
    {
        MUDA_GENERIC CDoublet(int index, const ValueT& segment)
            : index(index)
            , value(segment)
        {
        }
        int           index;
        const ValueT& value;
    };

    class Proxy
    {
        friend class DoubletVectorViewerT;
        const DoubletVectorViewerT& m_viewer;
        int                         m_index = 0;

      private:
        MUDA_GENERIC Proxy(const DoubletVectorViewerT& viewer, int index)
            : m_viewer(viewer)
            , m_index(index)
        {
        }

      public:
        MUDA_GENERIC auto read() && { return m_viewer.at(m_index); }

        MUDA_GENERIC void write(int segment_i, const ValueT& value) &&
        {
            auto index = m_viewer.get_index(m_index);

            m_viewer.check_in_subvector(segment_i);

            auto global_i = segment_i + m_viewer.m_subvector_offset;

            m_viewer.m_segment_indices[index] = global_i;
            m_viewer.m_segment_values[index]  = value;
        }

        MUDA_GENERIC ~Proxy() = default;
    };

  protected:
    // vector info
    int m_total_segment_count = 0;

    // doublet info
    int m_doublet_index_offset = 0;
    int m_doublet_count        = 0;
    int m_total_doublet_count  = 0;

    // subvector info
    int m_subvector_offset = 0;
    int m_subvector_extent = 0;

    // data
    auto_const_t<int>*    m_segment_indices;
    auto_const_t<ValueT>* m_segment_values;

  public:
    MUDA_GENERIC DoubletVectorViewerT() = default;
    MUDA_GENERIC DoubletVectorViewerT(int                total_segment_count,
                                      int                doublet_index_offset,
                                      int                doublet_count,
                                      int                total_doublet_count,
                                      int                subvector_offset,
                                      int                subvector_extent,
                                      auto_const_t<int>* segment_indices,
                                      auto_const_t<ValueT>* segment_values)
        : m_total_segment_count(total_segment_count)
        , m_doublet_index_offset(doublet_index_offset)
        , m_doublet_count(doublet_count)
        , m_total_doublet_count(total_doublet_count)
        , m_subvector_offset(subvector_offset)
        , m_subvector_extent(subvector_extent)
        , m_segment_indices(segment_indices)
        , m_segment_values(segment_values)
    {
        MUDA_KERNEL_ASSERT(doublet_index_offset + doublet_count <= total_doublet_count,
                           "DoubletVectorViewer: out of range, m_total_doublet_count=%d, "
                           "your doublet_index_offset=%d, doublet_count=%d. %s(%d)",
                           m_total_doublet_count,
                           doublet_index_offset,
                           doublet_count,
                           this->kernel_file(),
                           this->kernel_line());

        MUDA_KERNEL_ASSERT(subvector_offset + subvector_extent <= total_segment_count,
                           "DoubletVectorViewer: out of range, m_total_segment_count=%d, "
                           "your subvector_offset=%d, subvector_extent=%d. %s(%d)",
                           m_total_segment_count,
                           subvector_offset,
                           subvector_extent,
                           this->kernel_file(),
                           this->kernel_line());
    }

    template <bool OtherIsConst>
    MUDA_GENERIC DoubletVectorViewerT(const DoubletVectorViewerT<OtherIsConst, T, N>& other) noexcept
        MUDA_REQUIRES(IsConst)
        : m_total_segment_count(other.m_total_segment_count)
        , m_doublet_index_offset(other.m_doublet_index_offset)
        , m_doublet_count(other.m_doublet_count)
        , m_total_doublet_count(other.m_total_doublet_count)
        , m_subvector_offset(other.m_subvector_offset)
        , m_subvector_extent(other.m_subvector_extent)
        , m_segment_indices(other.m_segment_indices)
        , m_segment_values(other.m_segment_values)
    {
        static_assert(IsConst);
    }

    MUDA_GENERIC ConstViewer as_const() const noexcept
    {
        return ConstViewer{m_total_segment_count,
                           m_doublet_index_offset,
                           m_doublet_count,
                           m_total_doublet_count,
                           m_subvector_offset,
                           m_subvector_extent,
                           m_segment_indices,
                           m_segment_values};
    }

    MUDA_GENERIC int doublet_count() const noexcept { return m_doublet_count; }

    MUDA_GENERIC int total_doublet_count() const noexcept
    {
        return m_total_doublet_count;
    }

    MUDA_GENERIC auto operator()(int i) const
    {
        if constexpr(IsConst)
        {
            return at(i);
        }
        else
        {
            return Proxy{*this, i};
        }
    }

  protected:
    MUDA_INLINE MUDA_GENERIC CDoublet at(int i) const
    {
        auto index    = get_index(i);
        auto global_i = m_segment_indices[index];
        auto sub_i    = global_i - m_subvector_offset;

        check_in_subvector(sub_i);
        return CDoublet{sub_i, m_segment_values[index]};
    }


    MUDA_INLINE MUDA_GENERIC int get_index(int i) const noexcept
    {
        MUDA_KERNEL_ASSERT(i >= 0 && i < m_doublet_count,
                           "DoubletVectorViewer [%s:%s]: index out of range, m_doublet_count=%d, your index=%d. %s(%d)",
                           this->name(),
                           this->kernel_name(),
                           m_doublet_count,
                           i,
                           this->kernel_file(),
                           this->kernel_line());
        auto index = i + m_doublet_index_offset;
        return index;
    }

    MUDA_INLINE MUDA_GENERIC void check_in_subvector(int i) const noexcept
    {
        MUDA_KERNEL_ASSERT(i >= 0 && i < m_subvector_extent,
                           "DoubletVectorViewer [%s:%s]: index out of range, m_subvector_extent=%d, your index=%d. %s(%d)",
                           this->name(),
                           this->kernel_name(),
                           m_subvector_extent,
                           i,
                           this->kernel_file(),
                           this->kernel_line());
    }
};

template <typename T, int N>
using DoubletVectorViewer = DoubletVectorViewerT<false, T, N>;

template <typename T, int N>
using CDoubletVectorViewer = DoubletVectorViewerT<true, T, N>;
}  // namespace muda

#include "details/doublet_vector_viewer.inl"
