#pragma once
// View layer for the linear-system formats, ported from muda's
// ext/linear_system/*_view.h (+ their *_viewer.h) and viewer/dense/dense_{1d,2d}.h.
//
// Differences from muda:
//  * There is no separate viewer class: TripletMatrixViewer & co. are aliases
//    of the corresponding views, so the views also carry the viewer surface
//    (operator() with read/write proxies, segment/atomic_add, as_eigen, ...).
//  * No .name() / launch-label tracking. kernel_file()/kernel_line() exist
//    only as trivial stubs ("" / 0) because backend assert messages reference
//    them through the viewer aliases.
//  * DenseVectorView drops the cusparse descriptor and the stride member
//    (muda asserts inc==1 for every use the backend makes of it).
//  * The COOMatrixViewT/COOVectorViewBase cusparse-descriptor carriers are
//    not ported; BCOO*/CBCOO* are plain aliases of the triplet/doublet views.
//
// All view types are trivially copyable value types, safe to capture into
// kernels by value. No virtual functions anywhere.
#include <cuda_tool/view.h>
#include <cuda_tool/debug.h>
#include <cuda_tool/atomic.h>
#include <Eigen/Core>
#include <cstddef>

namespace uipc::backend::cuda_tool
{
// ---------------------------------------------------------------------------
// Triplet matrix view (COO, possibly block). Merges muda's TripletMatrixViewT
// and TripletMatrixViewerT into a single type.
// ---------------------------------------------------------------------------
template <bool IsConst, typename Ty, int M, int N = M>
class TripletMatrixViewT
{
    template <typename U>
    using auto_const_t = std::conditional_t<IsConst, const U, U>;

    template <bool OtherIsConst, typename U, int M_, int N_>
    friend class TripletMatrixViewT;

  public:
    static_assert(!std::is_const_v<Ty>, "Ty must be non-const");
    static constexpr bool IsBlockMatrix = (M > 1 || N > 1);
    using ValueT = std::conditional_t<IsBlockMatrix, Eigen::Matrix<Ty, M, N>, Ty>;

    using ConstView    = TripletMatrixViewT<true, Ty, M, N>;
    using NonConstView = TripletMatrixViewT<false, Ty, M, N>;
    using ThisView     = TripletMatrixViewT<IsConst, Ty, M, N>;

    // read-only triplet returned by operator() of a const view
    struct CTriplet
    {
        __host__ __device__ CTriplet(int row_index, int col_index, const ValueT& block)
            : row_index(row_index)
            , col_index(col_index)
            , value(block)
        {
        }

        int           row_index;
        int           col_index;
        const ValueT& value;
    };

    // write proxy returned by operator() of a non-const view
    class Proxy
    {
        friend class TripletMatrixViewT;
        const TripletMatrixViewT& m_viewer;
        int                       m_index = 0;

        __host__ __device__ Proxy(const TripletMatrixViewT& viewer, int index)
            : m_viewer(viewer)
            , m_index(index)
        {
        }

      public:
        __host__ __device__ CTriplet read() && { return m_viewer.at(m_index); }

        __host__ __device__ void write(int row_index,
                                       int col_index,
                                       const ValueT& block) &&
        {
            auto index = m_viewer.get_index(m_index);
            m_viewer.check_in_submatrix(row_index, col_index);

            auto global_i = m_viewer.m_submatrix_offset.x + row_index;
            auto global_j = m_viewer.m_submatrix_offset.y + col_index;

            m_viewer.m_row_indices[index] = global_i;
            m_viewer.m_col_indices[index] = global_j;
            m_viewer.m_values[index]      = block;
        }
    };

  protected:
    // matrix info
    int m_total_rows = 0;
    int m_total_cols = 0;

    // triplet info
    int m_triplet_index_offset = 0;
    int m_triplet_count        = 0;
    int m_total_triplet_count  = 0;

    // submatrix info
    int2 m_submatrix_offset = make_int2(0, 0);
    int2 m_submatrix_extent = make_int2(0, 0);

    // data
    auto_const_t<int>*    m_row_indices = nullptr;
    auto_const_t<int>*    m_col_indices = nullptr;
    auto_const_t<ValueT>* m_values      = nullptr;

  public:
    __host__ __device__ TripletMatrixViewT() = default;

    __host__ __device__ TripletMatrixViewT(int total_block_rows,
                                           int total_block_cols,
                                           int triplet_index_offset,
                                           int triplet_count,
                                           int total_triplet_count,
                                           int2 submatrix_offset,
                                           int2 submatrix_extent,
                                           auto_const_t<int>* row_indices,
                                           auto_const_t<int>* col_indices,
                                           auto_const_t<ValueT>* values)
        : m_total_rows(total_block_rows)
        , m_total_cols(total_block_cols)
        , m_triplet_index_offset(triplet_index_offset)
        , m_triplet_count(triplet_count)
        , m_total_triplet_count(total_triplet_count)
        , m_submatrix_offset(submatrix_offset)
        , m_submatrix_extent(submatrix_extent)
        , m_row_indices(row_indices)
        , m_col_indices(col_indices)
        , m_values(values)
    {
        UIPC_KERNEL_ASSERT(triplet_index_offset + triplet_count <= total_triplet_count,
                           "TripletMatrixView: out of range, total_triplet_count=%d, "
                           "your triplet_index_offset=%d, triplet_count=%d",
                           total_triplet_count,
                           triplet_index_offset,
                           triplet_count);

        UIPC_KERNEL_ASSERT(submatrix_offset.x >= 0 && submatrix_offset.y >= 0,
                           "TripletMatrixView: submatrix_offset is out of range, "
                           "submatrix_offset=(%d, %d)",
                           submatrix_offset.x,
                           submatrix_offset.y);

        UIPC_KERNEL_ASSERT(submatrix_offset.x + submatrix_extent.x <= total_block_rows,
                           "TripletMatrixView: submatrix is out of range, "
                           "submatrix_offset.x=%d, submatrix_extent.x=%d, "
                           "total_block_rows=%d",
                           submatrix_offset.x,
                           submatrix_extent.x,
                           total_block_rows);

        UIPC_KERNEL_ASSERT(submatrix_offset.y + submatrix_extent.y <= total_block_cols,
                           "TripletMatrixView: submatrix is out of range, "
                           "submatrix_offset.y=%d, submatrix_extent.y=%d, "
                           "total_block_cols=%d",
                           submatrix_offset.y,
                           submatrix_extent.y,
                           total_block_cols);
    }

    __host__ __device__ TripletMatrixViewT(int total_block_rows,
                                           int total_block_cols,
                                           int total_triplet_count,
                                           auto_const_t<int>*    block_row_indices,
                                           auto_const_t<int>*    block_col_indices,
                                           auto_const_t<ValueT>* block_values)
        : TripletMatrixViewT(total_block_rows,
                             total_block_cols,
                             0,
                             total_triplet_count,
                             total_triplet_count,
                             make_int2(0, 0),
                             make_int2(total_block_rows, total_block_cols),
                             block_row_indices,
                             block_col_indices,
                             block_values)
    {
    }

    // non-const -> const conversion
    template <bool OtherIsConst>
    __host__ __device__ TripletMatrixViewT(
        const TripletMatrixViewT<OtherIsConst, Ty, M, N>& other) noexcept
        requires(IsConst)
        : m_total_rows(other.m_total_rows)
        , m_total_cols(other.m_total_cols)
        , m_triplet_index_offset(other.m_triplet_index_offset)
        , m_triplet_count(other.m_triplet_count)
        , m_total_triplet_count(other.m_total_triplet_count)
        , m_submatrix_offset(other.m_submatrix_offset)
        , m_submatrix_extent(other.m_submatrix_extent)
        , m_row_indices(other.m_row_indices)
        , m_col_indices(other.m_col_indices)
        , m_values(other.m_values)
    {
        static_assert(IsConst);
    }

    __host__ __device__ ConstView as_const() const noexcept
    {
        return ConstView{m_total_rows,
                         m_total_cols,
                         m_triplet_index_offset,
                         m_triplet_count,
                         m_total_triplet_count,
                         m_submatrix_offset,
                         m_submatrix_extent,
                         m_row_indices,
                         m_col_indices,
                         m_values};
    }

    __host__ __device__ ThisView subview(int offset, int count) const
    {
        UIPC_KERNEL_ASSERT(offset + count <= m_triplet_count,
                           "TripletMatrixView: subview out of range, "
                           "triplet_count=%d, your offset=%d, your count=%d",
                           m_triplet_count,
                           offset,
                           count);

        return ThisView{m_total_rows,
                        m_total_cols,
                        m_triplet_index_offset + offset,
                        count,
                        m_total_triplet_count,
                        m_submatrix_offset,
                        m_submatrix_extent,
                        m_row_indices,
                        m_col_indices,
                        m_values};
    }

    __host__ __device__ ThisView subview(int offset) const
    {
        return subview(offset, m_triplet_count - offset);
    }

    __host__ __device__ ThisView submatrix(int2 offset, int2 extent) const
    {
        UIPC_KERNEL_ASSERT(offset.x >= 0 && offset.y >= 0,
                           "TripletMatrixView: submatrix is out of range, "
                           "offset=(%d, %d)",
                           offset.x,
                           offset.y);

        UIPC_KERNEL_ASSERT(offset.x + extent.x <= m_submatrix_extent.x
                               && offset.y + extent.y <= m_submatrix_extent.y,
                           "TripletMatrixView: submatrix is out of range, "
                           "offset=(%d, %d), extent=(%d, %d), "
                           "origin offset=(%d, %d), extent=(%d, %d)",
                           offset.x,
                           offset.y,
                           extent.x,
                           extent.y,
                           m_submatrix_offset.x,
                           m_submatrix_offset.y,
                           m_submatrix_extent.x,
                           m_submatrix_extent.y);

        return ThisView{m_total_rows,
                        m_total_cols,
                        m_triplet_index_offset,
                        m_triplet_count,
                        m_total_triplet_count,
                        make_int2(m_submatrix_offset.x + offset.x,
                                  m_submatrix_offset.y + offset.y),
                        extent,
                        m_row_indices,
                        m_col_indices,
                        m_values};
    }

    // viewers are the views themselves
    __host__ __device__ ThisView viewer() const noexcept { return *this; }

    __host__ __device__ ConstView cviewer() const noexcept { return as_const(); }

    __host__ __device__ int total_rows() const noexcept { return m_total_rows; }
    __host__ __device__ int total_cols() const noexcept { return m_total_cols; }
    __host__ __device__ int2 total_extent() const noexcept
    {
        return make_int2(m_total_rows, m_total_cols);
    }

    __host__ __device__ int2 submatrix_offset() const noexcept
    {
        return m_submatrix_offset;
    }
    __host__ __device__ int2 extent() const noexcept { return m_submatrix_extent; }

    __host__ __device__ int triplet_count() const noexcept { return m_triplet_count; }
    __host__ __device__ int tripet_index_offset() const noexcept
    {
        return m_triplet_index_offset;
    }
    __host__ __device__ int total_triplet_count() const noexcept
    {
        return m_total_triplet_count;
    }

    __host__ __device__ auto row_indices() const noexcept
    {
        return std::conditional_t<IsConst, CBufferView<int>, BufferView<int>>{
            m_row_indices,
            static_cast<size_t>(m_triplet_count),
            static_cast<size_t>(m_triplet_index_offset)};
    }

    __host__ __device__ auto col_indices() const noexcept
    {
        return std::conditional_t<IsConst, CBufferView<int>, BufferView<int>>{
            m_col_indices,
            static_cast<size_t>(m_triplet_count),
            static_cast<size_t>(m_triplet_index_offset)};
    }

    __host__ __device__ auto values() const noexcept
    {
        return std::conditional_t<IsConst, CBufferView<ValueT>, BufferView<ValueT>>{
            m_values,
            static_cast<size_t>(m_triplet_count),
            static_cast<size_t>(m_triplet_index_offset)};
    }

    // const view -> CTriplet (read); non-const view -> Proxy (read/write)
    __host__ __device__ auto operator()(int i) const
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

    // trivial stubs: backend assert messages reference these through the
    // viewer aliases; cuda_tool views do not track launch labels
    __host__ __device__ const char* kernel_file() const noexcept { return ""; }
    __host__ __device__ int kernel_line() const noexcept { return 0; }

  protected:
    __host__ __device__ CTriplet at(int i) const noexcept
    {
        auto index    = get_index(i);
        auto global_i = m_row_indices[index];
        auto global_j = m_col_indices[index];
        auto sub_i    = global_i - m_submatrix_offset.x;
        auto sub_j    = global_j - m_submatrix_offset.y;
        check_in_submatrix(sub_i, sub_j);
        return CTriplet{sub_i, sub_j, m_values[index]};
    }

    __host__ __device__ int get_index(int i) const noexcept
    {
        UIPC_KERNEL_ASSERT(i >= 0 && i < m_triplet_count,
                           "TripletMatrixView: triplet index out of range, "
                           "triplet_count=%d, your index=%d",
                           m_triplet_count,
                           i);
        return i + m_triplet_index_offset;
    }

    __host__ __device__ void check_in_submatrix(int i, int j) const noexcept
    {
        UIPC_KERNEL_ASSERT(i >= 0 && i < m_submatrix_extent.x,
                           "TripletMatrixView: row index out of submatrix range, "
                           "submatrix_extent.x=%d, your i=%d",
                           m_submatrix_extent.x,
                           i);
        UIPC_KERNEL_ASSERT(j >= 0 && j < m_submatrix_extent.y,
                           "TripletMatrixView: col index out of submatrix range, "
                           "submatrix_extent.y=%d, your j=%d",
                           m_submatrix_extent.y,
                           j);
    }
};

template <typename Ty, int M, int N = M>
using TripletMatrixView = TripletMatrixViewT<false, Ty, M, N>;
template <typename Ty, int M, int N = M>
using CTripletMatrixView = TripletMatrixViewT<true, Ty, M, N>;

// viewers are plain aliases of the views
template <typename Ty, int M, int N = M>
using TripletMatrixViewer = TripletMatrixView<Ty, M, N>;
template <typename Ty, int M, int N = M>
using CTripletMatrixViewer = CTripletMatrixView<Ty, M, N>;

// BCOO matrix views are triplet matrix views
template <typename T, int M, int N = M>
using BCOOMatrixView = TripletMatrixView<T, M, N>;
template <typename T, int M, int N = M>
using CBCOOMatrixView = CTripletMatrixView<T, M, N>;

// ---------------------------------------------------------------------------
// Doublet (sparse-segment) vector view. Merges muda's DoubletVectorViewT and
// DoubletVectorViewerT into a single type.
// ---------------------------------------------------------------------------
template <bool IsConst, typename T, int N>
class DoubletVectorViewT
{
    template <typename U>
    using auto_const_t = std::conditional_t<IsConst, const U, U>;

    template <bool OtherIsConst, typename U, int M>
    friend class DoubletVectorViewT;

  public:
    static_assert(!std::is_const_v<T>, "T must be non-const");
    static constexpr bool IsSegmentVector = (N > 1);
    using ValueT = std::conditional_t<IsSegmentVector, Eigen::Matrix<T, N, 1>, T>;

    using ConstView    = DoubletVectorViewT<true, T, N>;
    using NonConstView = DoubletVectorViewT<false, T, N>;
    using ThisView     = DoubletVectorViewT<IsConst, T, N>;

    // read-only doublet returned by operator() of a const view
    struct CDoublet
    {
        __host__ __device__ CDoublet(int index, const ValueT& segment)
            : index(index)
            , value(segment)
        {
        }

        int           index;
        const ValueT& value;
    };

    // write proxy returned by operator() of a non-const view
    class Proxy
    {
        friend class DoubletVectorViewT;
        const DoubletVectorViewT& m_viewer;
        int                       m_index = 0;

        __host__ __device__ Proxy(const DoubletVectorViewT& viewer, int index)
            : m_viewer(viewer)
            , m_index(index)
        {
        }

      public:
        __host__ __device__ CDoublet read() && { return m_viewer.at(m_index); }

        __host__ __device__ void write(int segment_i, const ValueT& value) &&
        {
            auto index = m_viewer.get_index(m_index);
            m_viewer.check_in_subvector(segment_i);

            auto global_i = segment_i + m_viewer.m_subvector_offset;

            m_viewer.m_indices[index] = global_i;
            m_viewer.m_values[index]  = value;
        }
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
    auto_const_t<int>*    m_indices = nullptr;
    auto_const_t<ValueT>* m_values  = nullptr;

  public:
    __host__ __device__ DoubletVectorViewT() = default;

    __host__ __device__ DoubletVectorViewT(int total_segment_count,
                                           int doublet_index_offset,
                                           int doublet_count,
                                           int total_doublet_count,
                                           int subvector_offset,
                                           int subvector_extent,
                                           auto_const_t<int>*    indices,
                                           auto_const_t<ValueT>* values)
        : m_total_segment_count(total_segment_count)
        , m_doublet_index_offset(doublet_index_offset)
        , m_doublet_count(doublet_count)
        , m_total_doublet_count(total_doublet_count)
        , m_subvector_offset(subvector_offset)
        , m_subvector_extent(subvector_extent)
        , m_indices(indices)
        , m_values(values)
    {
        UIPC_KERNEL_ASSERT(doublet_index_offset + doublet_count <= total_doublet_count,
                           "DoubletVectorView: out of range, total_doublet_count=%d, "
                           "your doublet_index_offset=%d, doublet_count=%d",
                           total_doublet_count,
                           doublet_index_offset,
                           doublet_count);

        UIPC_KERNEL_ASSERT(subvector_offset + subvector_extent <= total_segment_count,
                           "DoubletVectorView: out of range, total_segment_count=%d, "
                           "your subvector_offset=%d, subvector_extent=%d",
                           total_segment_count,
                           subvector_offset,
                           subvector_extent);
    }

    __host__ __device__ DoubletVectorViewT(int total_segment_count,
                                           int total_doublet_count,
                                           auto_const_t<int>*    segment_indices,
                                           auto_const_t<ValueT>* segment_values)
        : DoubletVectorViewT(total_segment_count,
                             0,
                             total_doublet_count,
                             total_doublet_count,
                             0,
                             total_segment_count,
                             segment_indices,
                             segment_values)
    {
    }

    // non-const -> const conversion
    template <bool OtherIsConst>
    __host__ __device__ DoubletVectorViewT(
        const DoubletVectorViewT<OtherIsConst, T, N>& other) noexcept
        requires(IsConst)
        : m_total_segment_count(other.m_total_segment_count)
        , m_doublet_index_offset(other.m_doublet_index_offset)
        , m_doublet_count(other.m_doublet_count)
        , m_total_doublet_count(other.m_total_doublet_count)
        , m_subvector_offset(other.m_subvector_offset)
        , m_subvector_extent(other.m_subvector_extent)
        , m_indices(other.m_indices)
        , m_values(other.m_values)
    {
        static_assert(IsConst);
    }

    __host__ __device__ ConstView as_const() const noexcept
    {
        return ConstView{m_total_segment_count,
                         m_doublet_index_offset,
                         m_doublet_count,
                         m_total_doublet_count,
                         m_subvector_offset,
                         m_subvector_extent,
                         m_indices,
                         m_values};
    }

    __host__ __device__ ThisView subview(int offset, int count) const noexcept
    {
        UIPC_KERNEL_ASSERT(offset + count <= m_doublet_count,
                           "DoubletVectorView: subview out of range, "
                           "doublet_count=%d, your offset=%d, your count=%d",
                           m_doublet_count,
                           offset,
                           count);

        return ThisView{m_total_segment_count,
                        m_doublet_index_offset + offset,
                        count,
                        m_total_doublet_count,
                        m_subvector_offset,
                        m_subvector_extent,
                        m_indices,
                        m_values};
    }

    __host__ __device__ ThisView subview(int offset) const noexcept
    {
        return subview(offset, m_doublet_count - offset);
    }

    __host__ __device__ ThisView subvector(int offset, int extent) const noexcept
    {
        UIPC_KERNEL_ASSERT(offset + extent <= m_subvector_extent,
                           "DoubletVectorView: subvector out of range, "
                           "subvector_extent=%d, your offset=%d, your extent=%d",
                           m_subvector_extent,
                           offset,
                           extent);

        return ThisView{m_total_segment_count,
                        m_doublet_index_offset,
                        m_doublet_count,
                        m_total_doublet_count,
                        m_subvector_offset + offset,
                        extent,
                        m_indices,
                        m_values};
    }

    // viewers are the views themselves
    __host__ __device__ ThisView viewer() const noexcept { return *this; }

    __host__ __device__ ConstView cviewer() const noexcept { return as_const(); }

    __host__ __device__ int extent() const noexcept { return m_subvector_extent; }

    __host__ __device__ int total_extent() const noexcept
    {
        return m_total_segment_count;
    }

    __host__ __device__ int subvector_offset() const noexcept
    {
        return m_subvector_offset;
    }

    __host__ __device__ int doublet_count() const noexcept { return m_doublet_count; }

    __host__ __device__ int total_doublet_count() const noexcept
    {
        return m_total_doublet_count;
    }

    __host__ __device__ auto indices() const noexcept
    {
        return std::conditional_t<IsConst, CBufferView<int>, BufferView<int>>{
            m_indices,
            static_cast<size_t>(m_doublet_count),
            static_cast<size_t>(m_doublet_index_offset)};
    }

    __host__ __device__ auto values() const noexcept
    {
        return std::conditional_t<IsConst, CBufferView<ValueT>, BufferView<ValueT>>{
            m_values,
            static_cast<size_t>(m_doublet_count),
            static_cast<size_t>(m_doublet_index_offset)};
    }

    // const view -> CDoublet (read); non-const view -> Proxy (read/write)
    __host__ __device__ auto operator()(int i) const
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

    // trivial stubs: backend assert messages reference these through the
    // viewer aliases; cuda_tool views do not track launch labels
    __host__ __device__ const char* kernel_file() const noexcept { return ""; }
    __host__ __device__ int kernel_line() const noexcept { return 0; }

  protected:
    __host__ __device__ CDoublet at(int i) const noexcept
    {
        auto index    = get_index(i);
        auto global_i = m_indices[index];
        auto sub_i    = global_i - m_subvector_offset;
        check_in_subvector(sub_i);
        return CDoublet{sub_i, m_values[index]};
    }

    __host__ __device__ int get_index(int i) const noexcept
    {
        UIPC_KERNEL_ASSERT(i >= 0 && i < m_doublet_count,
                           "DoubletVectorView: index out of range, "
                           "doublet_count=%d, your index=%d",
                           m_doublet_count,
                           i);
        return i + m_doublet_index_offset;
    }

    __host__ __device__ void check_in_subvector(int i) const noexcept
    {
        UIPC_KERNEL_ASSERT(i >= 0 && i < m_subvector_extent,
                           "DoubletVectorView: index out of subvector range, "
                           "subvector_extent=%d, your index=%d",
                           m_subvector_extent,
                           i);
    }
};

template <typename T, int N>
using DoubletVectorView = DoubletVectorViewT<false, T, N>;
template <typename T, int N>
using CDoubletVectorView = DoubletVectorViewT<true, T, N>;

// viewers are plain aliases of the views
template <typename T, int N>
using DoubletVectorViewer = DoubletVectorView<T, N>;
template <typename T, int N>
using CDoubletVectorViewer = CDoubletVectorView<T, N>;

// BCOO vector views are doublet vector views
template <typename T, int N>
using BCOOVectorView = DoubletVectorView<T, N>;
template <typename T, int N>
using CBCOOVectorView = CDoubletVectorView<T, N>;

// ---------------------------------------------------------------------------
// Dense vector view. Merges muda's DenseVectorViewT and DenseVectorViewerT
// into a single type; the cusparse descriptor and stride member are dropped
// (muda requires inc==1 for all of: cviewer()/descr()/subview()).
// ---------------------------------------------------------------------------
template <bool IsConst, typename T>
class DenseVectorViewT
{
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                  "only real numbers are supported");

    template <typename U>
    using auto_const_t = std::conditional_t<IsConst, const U, U>;

    template <bool OtherIsConst, typename U>
    friend class DenseVectorViewT;

  public:
    using ConstView    = DenseVectorViewT<true, T>;
    using NonConstView = DenseVectorViewT<false, T>;
    using ThisView     = DenseVectorViewT<IsConst, T>;

    using ThisBufferView = std::conditional_t<IsConst, CBufferView<T>, BufferView<T>>;

    using VectorType = Eigen::Vector<T, Eigen::Dynamic>;
    template <typename U>
    using MapVectorT =
        Eigen::Map<U, Eigen::Unaligned, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>;
    using MapVector     = MapVectorT<VectorType>;
    using CMapVector    = MapVectorT<const VectorType>;
    using ThisMapVector = std::conditional_t<IsConst, CMapVector, MapVector>;

  protected:
    auto_const_t<T>* m_data        = nullptr;
    int              m_offset      = 0;
    int              m_size        = 0;
    int              m_origin_size = 0;

  public:
    __host__ __device__ DenseVectorViewT() = default;

    __host__ __device__ DenseVectorViewT(auto_const_t<T>* data,
                                         int              offset,
                                         int              size,
                                         int              origin_size)
        : m_data(data)
        , m_offset(offset)
        , m_size(size)
        , m_origin_size(origin_size)
    {
    }

    // non-const -> const conversion
    template <bool OtherIsConst>
    __host__ __device__ DenseVectorViewT(const DenseVectorViewT<OtherIsConst, T>& other) noexcept
        requires(IsConst)
        : m_data(other.m_data)
        , m_offset(other.m_offset)
        , m_size(other.m_size)
        , m_origin_size(other.m_origin_size)
    {
        static_assert(IsConst);
    }

    __host__ __device__ ConstView as_const() const noexcept
    {
        return ConstView{m_data, m_offset, m_size, m_origin_size};
    }

    // viewers are the views themselves
    __host__ __device__ ThisView viewer() const noexcept { return *this; }

    __host__ __device__ ConstView cviewer() const noexcept { return as_const(); }

    __host__ __device__ ThisBufferView buffer_view() const noexcept
    {
        return ThisBufferView{m_data,
                              static_cast<size_t>(m_size),
                              static_cast<size_t>(m_offset)};
    }

    __host__ __device__ auto_const_t<T>* data() const noexcept
    {
        return m_data + m_offset;
    }

    __host__ __device__ auto_const_t<T>* origin_data() const noexcept { return m_data; }

    __host__ __device__ int offset() const noexcept { return m_offset; }

    __host__ __device__ int size() const noexcept { return m_size; }

    __host__ __device__ int origin_size() const noexcept { return m_origin_size; }

    __host__ __device__ ThisView subview(int offset, int size) const
    {
        UIPC_KERNEL_ASSERT(offset >= 0 && offset + size <= m_size,
                           "DenseVectorView: subview out of range, "
                           "size=%d, your offset=%d, your size=%d",
                           m_size,
                           offset,
                           size);
        return ThisView{m_data, m_offset + offset, size, m_origin_size};
    }

    __host__ __device__ ThisView segment(int offset, int size) const
    {
        return subview(offset, size);
    }

    template <int SegN>
    __host__ __device__ ThisView segment(int offset) const
    {
        return subview(offset, SegN);
    }

    __host__ __device__ auto_const_t<T>& operator()(int i) const noexcept
    {
        UIPC_KERNEL_ASSERT(m_data != nullptr, "DenseVectorView: data is null");
        UIPC_KERNEL_ASSERT(i >= 0 && i < m_size,
                           "DenseVectorView: index out of range, "
                           "size=%d, your index=%d",
                           m_size,
                           i);
        return m_data[m_offset + i];
    }

    // element-wise assignment from an Eigen vector/expression of matching
    // size (non-const views only), mirroring muda's DenseVectorViewer
    template <typename Derived>
    __host__ __device__ ThisView& operator=(const Eigen::MatrixBase<Derived>& expr)
        requires(!IsConst)
    {
        UIPC_KERNEL_ASSERT(expr.size() == m_size,
                           "DenseVectorView: assignment size mismatch, "
                           "size=%d, your size=%d",
                           m_size,
                           (int)expr.size());
        for(int i = 0; i < m_size; ++i)
            (*this)(i) = expr.derived()(i);
        return *this;
    }

    __host__ __device__ Eigen::VectorBlock<ThisMapVector> as_eigen() const
    {
        UIPC_KERNEL_ASSERT(m_data != nullptr, "DenseVectorView: data is null");
        return ThisMapVector{m_data,
                             m_origin_size,
                             Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>{1, 1}}
            .segment(m_offset, m_size);
    }

    // device-side atomic add (non-const views only); returns the old value
    __device__ T atomic_add(int i, T val) const requires(!IsConst)
    {
        return cuda_tool::atomic_add(&this->operator()(i), val);
    }

    template <int SegN>
    __device__ Eigen::Vector<T, SegN> atomic_add(const Eigen::Vector<T, SegN>& val) const
        requires(!IsConst)
    {
        UIPC_KERNEL_ASSERT(m_size == SegN,
                           "DenseVectorView: size mismatch, size=%d, your size=%d",
                           m_size,
                           (int)SegN);
        Eigen::Vector<T, SegN> ret;
#pragma unroll
        for(int i = 0; i < SegN; ++i)
            ret(i) = atomic_add(i, val(i));
        return ret;
    }

    __device__ T atomic_add(const T& val) const requires(!IsConst)
    {
        UIPC_KERNEL_ASSERT(m_size == 1,
                           "DenseVectorView: size mismatch, size=%d, expected 1",
                           m_size);
        return atomic_add(0, val);
    }
};

template <typename T>
using DenseVectorView = DenseVectorViewT<false, T>;
template <typename T>
using CDenseVectorView = DenseVectorViewT<true, T>;

// viewers are plain aliases of the views
template <typename T>
using DenseVectorViewer = DenseVectorView<T>;
template <typename T>
using CDenseVectorViewer = CDenseVectorView<T>;

}  // namespace uipc::backend::cuda_tool
