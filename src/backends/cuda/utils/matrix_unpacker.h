#pragma once
#include <type_define.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
template <typename T, int BlockDim>
class TripletMatrixUnpacker
{
  public:
    UIPC_GENERIC TripletMatrixUnpacker(const cuda_tool::TripletMatrixViewer<T, BlockDim>& triplet)
        : m_triplet(triplet)
    {
    }

    template <int M, int N>
        requires(M >= 1 && N >= 1)
    class ProxyRange
    {
      public:
        UIPC_GENERIC ProxyRange(const TripletMatrixUnpacker& unpacker, IndexT I)
            : m_unpacker(unpacker)
            , m_I(I)
        {
            UIPC_KERNEL_ASSERT(I + (M * N) <= m_unpacker.m_triplet.triplet_count(),
                        "Triplet out of range, I = %d, Count=%d * %d, total=%d. %s(%d)",
                        I,
                        M,
                        N,
                        m_unpacker.m_triplet.triplet_count(),
                        m_unpacker.m_triplet.kernel_file(),
                        m_unpacker.m_triplet.kernel_line());
        }


        UIPC_GENERIC void write(IndexT i,
                                IndexT j,
                                const Eigen::Matrix<T, BlockDim * M, BlockDim * N>& value)
            requires(BlockDim > 1 && (M > 1 || N > 1))
        {
            IndexT offset = m_I;
            for(IndexT ii = 0; ii < M; ++ii)
            {
                for(IndexT jj = 0; jj < N; ++jj)
                {
                    m_unpacker.m_triplet(offset).write(
                        i + ii,
                        j + jj,
                        value.template block<BlockDim, BlockDim>(ii * BlockDim, jj * BlockDim));
                    ++offset;
                }
            }
        }

        UIPC_GENERIC void write(IndexT i, IndexT j, const Eigen::Matrix<T, M, N>& value)
            requires(BlockDim == 1 && (M > 1 || N > 1))
        {
            IndexT offset = m_I;
            for(IndexT ii = 0; ii < M; ++ii)
            {
                for(IndexT jj = 0; jj < N; ++jj)
                {
                    m_unpacker.m_triplet(offset).write(i + ii, j + jj, value(ii, jj));
                    ++offset;
                }
            }
        }


        UIPC_GENERIC void write(IndexT i, IndexT j, const Eigen::Matrix<T, BlockDim, BlockDim>& value)
            requires(BlockDim > 1 && M == 1 && N == 1)
        {
            m_unpacker.m_triplet(m_I).write(i, j, value);
        }


        UIPC_GENERIC void write(IndexT i, IndexT j, const T& value)
            requires(BlockDim == 1 && M == 1 && N == 1)
        {
            m_unpacker.m_triplet(m_I).write(i, j, value);
        }


      private:
        const TripletMatrixUnpacker& m_unpacker;
        IndexT                       m_I;
    };

    template <int N>
        requires(N >= 1)
    class ProxyRangeHalf
    {
      public:
        struct UpperIJ
        {
            IndexT dst_i;
            IndexT dst_j;
            IndexT ii;
            IndexT jj;
        };

        using BlockMatrix = Eigen::Matrix<T, N * BlockDim, N * BlockDim>;
        UIPC_GENERIC ProxyRangeHalf(const TripletMatrixUnpacker& unpacker, IndexT I)
            : m_unpacker(unpacker)
            , m_I(I)
        {
            UIPC_KERNEL_ASSERT(I + (N * (N + 1)) / 2 <= m_unpacker.m_triplet.triplet_count(),
                        "Triplet out of range, I = %d, Count=%d * %d / 2, total=%d. %s(%d)",
                        I,
                        N,
                        N,
                        m_unpacker.m_triplet.triplet_count(),
                        m_unpacker.m_triplet.kernel_file(),
                        m_unpacker.m_triplet.kernel_line());
        }

        /**
         * @brief Only write to the upper triangular part of the global matrix. (not the submatrix)
         */
        UIPC_GENERIC void write(IndexT i, IndexT j, const BlockMatrix& value)
        {
            IndexT offset = m_I;
            for(IndexT ii = 0; ii < N; ++ii)
            {
                for(IndexT jj = ii; jj < N; ++jj)
                {

                    auto [dst_i, dst_j, ii_, jj_] = upper_ij(i, j, ii, jj);

                    m_unpacker.m_triplet(offset++).write(
                        dst_i,
                        dst_j,
                        value.template block<BlockDim, BlockDim>(ii_ * BlockDim, jj_ * BlockDim));
                }
            }
        }

      private:
        UIPC_GENERIC UpperIJ upper_ij(const IndexT& i,
                                      const IndexT& j,
                                      const IndexT& ii,
                                      const IndexT& jj)
        {
            auto submatrix_offset = m_unpacker.m_triplet.submatrix_offset();
            UIPC_KERNEL_ASSERT(submatrix_offset.x == submatrix_offset.y,
                        "Symmetric assembly requires a square submatrix view, but your submatrix offset.x=%d, submatrix_offset.y=%d",
                        submatrix_offset.x,
                        submatrix_offset.y);
            UpperIJ ret;
            // keep it in upper triangular in the global matrix (not the submatrix)
            auto dst_i = (i + ii);
            auto dst_j = (j + jj);
            if(dst_i + submatrix_offset.x < dst_j + submatrix_offset.y)
            {
                ret.dst_i = dst_i;
                ret.dst_j = dst_j;
                ret.ii    = ii;
                ret.jj    = jj;
            }
            else
            {
                ret.dst_i = dst_j;
                ret.dst_j = dst_i;
                ret.ii    = jj;
                ret.jj    = ii;
            }
            return ret;
        }

        const TripletMatrixUnpacker& m_unpacker;
        IndexT                       m_I;
    };

    /** 
     * @brief Take a range of [I, I + M * N) from the triplets.
     */
    template <int M, int N>
    UIPC_GENERIC ProxyRange<M, N> block(IndexT I) const
    {
        return ProxyRange<M, N>(*this, I);
    }

    template <int N>
    UIPC_GENERIC ProxyRangeHalf<N> half_block(IndexT I) const
    {
        return ProxyRangeHalf<N>(*this, I);
    }

    /**
     * @brief Take a range of [I, I + N) from the triplets.
     */
    template <int N>
    UIPC_GENERIC ProxyRange<N, 1> segment(IndexT I) const
    {
        return ProxyRange<N, 1>(*this, I);
    }

    /** 
     * @brief Take a range of [I, I + 1) from the triplets.
     */
    UIPC_GENERIC ProxyRange<1, 1> operator()(IndexT I) const
    {
        return ProxyRange<1, 1>(*this, I);
    }

  private:
    const cuda_tool::TripletMatrixViewer<T, BlockDim>& m_triplet;
};

// CTAD
template <typename T, int BlockDim>
TripletMatrixUnpacker(const cuda_tool::TripletMatrixViewer<T, BlockDim>&)
    -> TripletMatrixUnpacker<T, BlockDim>;
}  // namespace uipc::backend::cuda