#pragma once
#include <type_define.h>
#include <muda/ext/linear_system.h>

namespace uipc::backend::cuda
{
template <typename T, int BlockDim>
class TripletMatrixUnpacker
{
  public:
    MUDA_GENERIC TripletMatrixUnpacker(muda::TripletMatrixViewer<T, BlockDim>& triplet)
        : m_triplet(triplet)
    {
    }

    template <int M, int N>
        requires(M >= 1 && N >= 1)
    class ProxyRange
    {
      public:
        MUDA_GENERIC ProxyRange(TripletMatrixUnpacker& unpacker, IndexT I)
            : m_unpacker(unpacker)
            , m_I(I)
        {
            MUDA_ASSERT(I + (M * N) <= m_unpacker.m_triplet.triplet_count(),
                        "Triplet out of range, I = %d, Count=%d * %d, total=%d. %s(%d)",
                        I,
                        M,
                        N,
                        m_unpacker.m_triplet.triplet_count(),
                        m_unpacker.m_triplet.kernel_file(),
                        m_unpacker.m_triplet.kernel_line());
        }


        MUDA_GENERIC void write(IndexT i,
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

        MUDA_GENERIC void write(IndexT i, IndexT j, const Eigen::Matrix<T, M, N>& value)
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


        MUDA_GENERIC void write(IndexT i, IndexT j, const Eigen::Matrix<T, BlockDim, BlockDim>& value)
            requires(BlockDim > 1 && M == 1 && N == 1)
        {
            m_unpacker.m_triplet(m_I).write(i, j, value);
        }


        MUDA_GENERIC void write(IndexT i, IndexT j, const T& value)
            requires(BlockDim == 1 && M == 1 && N == 1)
        {
            m_unpacker.m_triplet(m_I).write(i, j, value);
        }


      private:
        TripletMatrixUnpacker& m_unpacker;
        IndexT                 m_I;
    };

    /** 
     * @brief Take a range of [I, I + M * N) from the triplets.
     */
    template <int M, int N>
    MUDA_GENERIC ProxyRange<M, N> block(IndexT I)
    {
        return ProxyRange<M, N>(*this, I);
    }

    /**
     * @brief Take a range of [I, I + N) from the triplets.
     */
    template <int N>
    MUDA_GENERIC ProxyRange<N, 1> segment(IndexT I)
    {
        return ProxyRange<N, 1>(*this, I);
    }

    /** 
     * @brief Take a range of [I, I + 1) from the triplets.
     */
    MUDA_GENERIC ProxyRange<1, 1> operator()(IndexT I)
    {
        return ProxyRange<1, 1>(*this, I);
    }

  private:
    muda::TripletMatrixViewer<T, BlockDim>& m_triplet;
};

// CTAD
template <typename T, int BlockDim>
TripletMatrixUnpacker(muda::TripletMatrixViewer<T, BlockDim>&)
    -> TripletMatrixUnpacker<T, BlockDim>;
}  // namespace uipc::backend::cuda