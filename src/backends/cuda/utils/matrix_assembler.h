#pragma once
#include <type_define.h>
#include <muda/ext/linear_system.h>

namespace uipc::backend::cuda
{
template <int M, int N>
UIPC_GENERIC void zero_out(Vector<Float, M>& Vec, const Vector<IndexT, N>& zero_out_flag)
    requires(M % N == 0)
{
    constexpr int Segment = M / N;
    for(int i = 0; i < N; ++i)
    {
        if(zero_out_flag(i))
            Vec.template segment<Segment>(i * Segment).setZero();
    }
}

template <int M, int N>
UIPC_GENERIC void zero_out(Matrix<Float, M, M>& Mat, const Vector<IndexT, N>& zero_out_flag)
    requires(M % N == 0)
{
    constexpr int Segment = M / N;
    for(int i = 0; i < N; ++i)
    {
        for(int j = 0; j < N; ++j)
        {
            if(zero_out_flag(i) || zero_out_flag(j))
                Mat.template block<Segment, Segment>(i * Segment, j * Segment).setZero();
        }
    }
}
template <typename T>
class DenseVectorAssembler
{
  public:
    MUDA_GENERIC DenseVectorAssembler(const muda::DenseVectorViewer<T>& dense)
        : m_dense(dense)
    {
    }

    template <int M, int N>
    MUDA_DEVICE void atomic_add(const Vector<IndexT, N>& indices,
                                const Vector<IndexT, N>& ignore,
                                const Vector<Float, M>&  G3N)
        requires(N >= 2 && M % N == 0)
    {
        constexpr int SegmentDim = M / N;
        using SegmentVector      = Vector<Float, SegmentDim>;
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
            int dst = indices(i);
            if(ignore(i))
                continue;
            SegmentVector G = G3N.template segment<SegmentDim>(i * SegmentDim);

            m_dense
                .template segment<SegmentDim>(dst * SegmentDim)  //
                .template atomic_add<SegmentDim>(G);
        }
    }

    template <int M, int N>
    MUDA_DEVICE void atomic_add(const Vector<IndexT, N>& indices,
                                const Vector<Float, M>&  G3N)
        requires(N >= 2 && M % N == 0)
    {
        constexpr int SegmentDim = M / N;
        using SegmentVector      = Vector<Float, SegmentDim>;
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
            int           dst = indices(i);
            SegmentVector G = G3N.template segment<SegmentDim>(i * SegmentDim);

            m_dense
                .template segment<SegmentDim>(dst * SegmentDim)  //
                .template atomic_add<SegmentDim>(G);
        }
    }

  private:
    const muda::DenseVectorViewer<T>& m_dense;
};

// CTAD
template <typename T>
DenseVectorAssembler(const muda::DenseVectorViewer<T>&) -> DenseVectorAssembler<T>;

template <typename T, int SegmentDim>
class DoubletVectorAssembler
{
  public:
    using ElementVector = Eigen::Matrix<T, SegmentDim, 1>;

    MUDA_GENERIC DoubletVectorAssembler(const muda::DoubletVectorViewer<T, SegmentDim>& doublet)
        : m_doublet(doublet)
    {
    }

    template <int N>
        requires(N >= 1)
    class ProxyRange
    {
      public:
        using SegmentVector = Eigen::Vector<T, N * SegmentDim>;

        MUDA_GENERIC ProxyRange(DoubletVectorAssembler& assembler, IndexT I)
            : m_assembler(assembler)
            , m_I(I)
        {
            MUDA_ASSERT(I + N <= m_assembler.m_doublet.doublet_count(),
                        "Doublet out of range, I = %d, Count=%d, total=%d. %s(%d)",
                        I,
                        N,
                        m_assembler.m_doublet.doublet_count(),
                        m_assembler.m_doublet.kernel_file(),
                        m_assembler.m_doublet.kernel_line());
        }

        MUDA_GENERIC void write(const Eigen::Vector<IndexT, N>& indices,
                                const SegmentVector&            value)
            requires(N > 1)
        {
            IndexT offset = m_I;
            for(IndexT ii = 0; ii < N; ++ii)
            {
                ElementVector G = value.template segment<SegmentDim>(ii * SegmentDim);
                m_assembler.m_doublet(offset++).write(indices(ii), G);
            }
        }

        MUDA_GENERIC void write(const Eigen::Vector<IndexT, N>& indices,
                                const Eigen::Vector<IndexT, N>& ignore,
                                const SegmentVector&            value)
            requires(N > 1)
        {
            IndexT offset = m_I;
            for(IndexT ii = 0; ii < N; ++ii)
            {
                ElementVector G = value.template segment<SegmentDim>(ii * SegmentDim);
                if(ignore(ii))
                    G.setZero();
                m_assembler.m_doublet(offset++).write(indices(ii), G);
            }
        }

        MUDA_GENERIC void write(IndexT indices, const ElementVector& value)
            requires(N == 1)
        {
            m_assembler.m_doublet(m_I).write(indices, value);
        }

        MUDA_GENERIC void write(IndexT indices, IndexT ignore, const ElementVector& value)
            requires(N == 1)
        {
            ElementVector G = value;
            if(ignore)
                G.setZero();
            m_assembler.m_doublet(m_I).write(indices, G);
        }

      private:
        const DoubletVectorAssembler& m_assembler;
        IndexT                        m_I;
    };


    /** 
     * @brief Take a range of [I, I + N) from the doublets.
     */
    template <int N>
    MUDA_GENERIC ProxyRange<N> segment(IndexT I)
    {
        return ProxyRange<N>(*this, I);
    }

    /** 
     * @brief Take a range of [I, I + 1) from the doublets.
     */
    MUDA_GENERIC ProxyRange<1> operator()(IndexT I)
    {
        return ProxyRange<1>(*this, I);
    }

  private:
    const muda::DoubletVectorViewer<T, SegmentDim>& m_doublet;
};

// CTAD
template <typename T, int SegmentDim>
DoubletVectorAssembler(muda::DoubletVectorViewer<T, SegmentDim>&)
    -> DoubletVectorAssembler<T, SegmentDim>;


template <typename T, int BlockDim>
class TripletMatrixAssembler
{
  public:
    using ElementMatrix = Eigen::Matrix<T, BlockDim, BlockDim>;


    MUDA_GENERIC TripletMatrixAssembler(muda::TripletMatrixViewer<T, BlockDim>& triplet)
        : m_triplet(triplet)
    {
    }

    template <int N>
        requires(N >= 1)
    class ProxyRange
    {
      public:
        using BlockMatrix = Eigen::Matrix<T, N * BlockDim, N * BlockDim>;

        MUDA_GENERIC ProxyRange(TripletMatrixAssembler& assembler, IndexT I)
            : m_assembler(assembler)
            , m_I(I)
        {
            MUDA_ASSERT(I + (N * N) <= m_assembler.m_triplet.triplet_count(),
                        "Triplet out of range, I = %d, Count=%d * %d, total=%d. %s(%d)",
                        I,
                        N,
                        N,
                        m_assembler.m_triplet.triplet_count(),
                        m_assembler.m_triplet.kernel_file(),
                        m_assembler.m_triplet.kernel_line());
        }


        MUDA_GENERIC void write(const Eigen::Vector<IndexT, N>& l_indices,
                                const Eigen::Vector<IndexT, N>& r_indices,
                                const BlockMatrix&              value)
            requires(N > 1)
        {
            IndexT offset = m_I;
            for(IndexT ii = 0; ii < N; ++ii)
            {
                for(IndexT jj = 0; jj < N; ++jj)
                {
                    ElementMatrix H =
                        value.template block<BlockDim, BlockDim>(ii * BlockDim, jj * BlockDim);

                    m_assembler.m_triplet(offset++).write(l_indices(ii), r_indices(jj), H);
                }
            }
        }

        MUDA_GENERIC void write(const Eigen::Vector<IndexT, N>& l_indices,
                                const Eigen::Vector<int8_t, N>& l_ignore,
                                const Eigen::Vector<IndexT, N>& r_indices,
                                const Eigen::Vector<int8_t, N>& r_ignore,
                                const BlockMatrix&              value)
            requires(N > 1)
        {
            IndexT offset = m_I;
            for(IndexT ii = 0; ii < N; ++ii)
            {
                for(IndexT jj = 0; jj < N; ++jj)
                {
                    ElementMatrix H;
                    if(l_ignore(ii) || r_ignore(jj))
                        H.setZero();
                    else
                        H = value.template block<BlockDim, BlockDim>(ii * BlockDim,
                                                                     jj * BlockDim);


                    m_assembler.m_triplet(offset++).write(l_indices(ii), r_indices(jj), H);
                }
            }
        }


        MUDA_GENERIC void write(const Eigen::Vector<IndexT, N>& indices, const BlockMatrix& value)
            requires(N > 1)
        {
            write(indices, indices, value);
        }


        MUDA_GENERIC void write(const Eigen::Vector<IndexT, N>& indices,
                                const Eigen::Vector<int8_t, N>  ignore,
                                const BlockMatrix&              value)
            requires(N > 1)
        {
            write(indices, ignore, indices, ignore, value);
        }

        MUDA_GENERIC void write(IndexT indices, const ElementMatrix& value)
            requires(N == 1)
        {
            IndexT offset = m_I;
            m_assembler.m_triplet(offset).write(indices, indices, value);
        }

        MUDA_GENERIC void write(IndexT indices, IndexT ignore, const ElementMatrix& value)
            requires(N == 1)
        {
            IndexT        offset = m_I;
            ElementMatrix H      = value;
            if(ignore)
                H.setZero();
            m_assembler.m_triplet(offset).write(indices, indices, H);
        }

      private:
        const TripletMatrixAssembler& m_assembler;
        IndexT                        m_I;
    };

    template <int N>
        requires(N >= 1)
    class ProxyRangeHalf
    {
      public:
        struct UpperLR
        {
            IndexT L;
            IndexT R;
        };

        using BlockMatrix = Eigen::Matrix<T, N * BlockDim, N * BlockDim>;
        MUDA_GENERIC ProxyRangeHalf(const TripletMatrixAssembler& assembler, IndexT I)
            : m_assembler(assembler)
            , m_I(I)
        {
            MUDA_ASSERT(I + (N * (N + 1)) / 2 <= m_assembler.m_triplet.triplet_count(),
                        "Triplet out of range, I = %d, Count=%d * %d / 2, total=%d. %s(%d)",
                        I,
                        N,
                        N,
                        m_assembler.m_triplet.triplet_count(),
                        m_assembler.m_triplet.kernel_file(),
                        m_assembler.m_triplet.kernel_line());
        }

        /**
         * @brief Only write to the upper triangular part of the global matrix. (not the submatrix)
         */
        MUDA_GENERIC void write(const Eigen::Vector<IndexT, N>& indices, const BlockMatrix& value)
        {
            IndexT offset = m_I;
            for(IndexT ii = 0; ii < N; ++ii)
            {
                for(IndexT jj = ii; jj < N; ++jj)
                {

                    auto [L, R] = upper_LR(indices, ii, jj);

                    ElementMatrix H =
                        value.template block<BlockDim, BlockDim>(L * BlockDim, R * BlockDim);

                    m_assembler.m_triplet(offset++).write(indices(L), indices(R), H);
                }
            }
        }

        /**
         * @brief Only write to the upper triangular part of the global matrix. (not the submatrix)
         */
        MUDA_GENERIC void write(const Eigen::Vector<IndexT, N>& l_indices,
                                const Eigen::Vector<IndexT, N>& r_indices,
                                const BlockMatrix&              value)
        {
            IndexT offset = m_I;
            for(IndexT ii = 0; ii < N; ++ii)
            {
                for(IndexT jj = ii; jj < N; ++jj)
                {

                    auto [L, R] = upper_LR(l_indices, r_indices, ii, jj);

                    ElementMatrix H =
                        value.template block<BlockDim, BlockDim>(L * BlockDim, R * BlockDim);

                    m_assembler.m_triplet(offset++).write(l_indices(L), r_indices(R), H);
                }
            }
        }

        /**
         * @brief Only write to the upper triangular part of the global matrix. (not the submatrix)
         * 
         * Constraints: if either side is ignored, write zero.
         */
        MUDA_GENERIC void write(const Eigen::Vector<IndexT, N>& indices,
                                const Eigen::Vector<int8_t, N>  ignore,
                                const BlockMatrix&              value)
        {
            IndexT offset = m_I;
            for(IndexT ii = 0; ii < N; ++ii)
            {
                for(IndexT jj = ii; jj < N; ++jj)
                {
                    auto [L, R] = upper_LR(indices, ii, jj);

                    ElementMatrix H;
                    if(ignore(L) || ignore(R))
                    {
                        H.setZero();
                    }
                    else
                    {
                        H = value.template block<BlockDim, BlockDim>(L * BlockDim, R * BlockDim);
                    }
                    m_assembler.m_triplet(offset++).write(indices(L), indices(R), H);
                }
            }
        }

      private:
        MUDA_GENERIC UpperLR upper_LR(const Eigen::Vector<IndexT, N>& indices,
                                      const IndexT&                   I,
                                      const IndexT&                   J)
        {
            return upper_LR(indices, indices, I, J);
        }

        MUDA_GENERIC UpperLR upper_LR(const Eigen::Vector<IndexT, N>& l_indices,
                                      const Eigen::Vector<IndexT, N>& r_indices,
                                      const IndexT&                   I,
                                      const IndexT&                   J)
        {
            auto submatrix_offset = m_assembler.m_triplet.submatrix_offset();
            MUDA_ASSERT(submatrix_offset.x == submatrix_offset.y,
                        "Symmetric assembly requires a square submatrix view, but your submatrix offset.x=%d, submatrix_offset.y=%d",
                        submatrix_offset.x,
                        submatrix_offset.y);
            UpperLR ret;
            // keep it in upper triangular in the global matrix (not the submatrix)
            if(l_indices(I) + submatrix_offset.x
               < r_indices(J) + submatrix_offset.y)
            {
                ret.L = I;
                ret.R = J;
            }
            else
            {
                ret.L = J;
                ret.R = I;
            }
            return ret;
        }

        const TripletMatrixAssembler& m_assembler;
        IndexT                        m_I;
    };


    /** 
     * @brief Take a range of [I, I + N * N) from the triplets.
     */
    template <int M, int N>
    MUDA_GENERIC ProxyRange<N> block(IndexT I)
        requires(M == N)
    {
        return ProxyRange<N>(*this, I);
    }

    /** 
     * @brief Take a range of [I, I + N * (N + 1) / 2) from the triplets.
     */
    template <int N>
    MUDA_GENERIC ProxyRangeHalf<N> half_block(IndexT I)
    {
        return ProxyRangeHalf<N>(*this, I);
    }

    /** 
     * @brief Take a range of [I, I + 1) from the triplets.
     */
    MUDA_GENERIC ProxyRange<1> operator()(IndexT I)
    {
        return ProxyRange<1>(*this, I);
    }

  private:
    const muda::TripletMatrixViewer<T, BlockDim>& m_triplet;
};

// CTAD
template <typename T, int BlockDim>
TripletMatrixAssembler(const muda::TripletMatrixViewer<T, BlockDim>&)
    -> TripletMatrixAssembler<T, BlockDim>;
}  // namespace uipc::backend::cuda