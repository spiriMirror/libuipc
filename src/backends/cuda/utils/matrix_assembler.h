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
    MUDA_GENERIC DenseVectorAssembler(muda::DenseVectorViewer<T>& dense)
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
    muda::DenseVectorViewer<T>& m_dense;
};

// CTAD
template <typename T>
DenseVectorAssembler(muda::DenseVectorViewer<T>&) -> DenseVectorAssembler<T>;

template <typename T, int SegmentDim>
class DoubletVectorAssembler
{
  public:
    using ElementVector = Eigen::Matrix<T, SegmentDim, 1>;

    MUDA_GENERIC DoubletVectorAssembler(muda::DoubletVectorViewer<T, SegmentDim>& doublet)
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
        DoubletVectorAssembler& m_assembler;
        IndexT                  m_I;
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
    muda::DoubletVectorViewer<T, SegmentDim>& m_doublet;
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


        MUDA_GENERIC void write(const Eigen::Vector<IndexT, N>& indices, const BlockMatrix& value)
            requires(N > 1)
        {
            IndexT offset = m_I;
            for(IndexT ii = 0; ii < N; ++ii)
            {
                for(IndexT jj = 0; jj < N; ++jj)
                {
                    ElementMatrix H =
                        value.template block<BlockDim, BlockDim>(ii * BlockDim, jj * BlockDim);

                    m_assembler.m_triplet(offset++).write(indices(ii), indices(jj), H);
                }
            }
        }

        MUDA_GENERIC void write(const Eigen::Vector<IndexT, N>& indices,
                                const Eigen::Vector<IndexT, N>& ignore,
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
                    if(ignore(ii) || ignore(jj))
                        H.setZero();

                    m_assembler.m_triplet(offset++).write(indices(ii), indices(jj), H);
                }
            }
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
        TripletMatrixAssembler& m_assembler;
        IndexT                  m_I;
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
     * @brief Take a range of [I, I + 1) from the triplets.
     */
    MUDA_GENERIC ProxyRange<1> operator()(IndexT I)
    {
        return ProxyRange<1>(*this, I);
    }

  private:
    muda::TripletMatrixViewer<T, BlockDim>& m_triplet;
};

// CTAD
template <typename T, int BlockDim>
TripletMatrixAssembler(muda::TripletMatrixViewer<T, BlockDim>&)
    -> TripletMatrixAssembler<T, BlockDim>;
}  // namespace uipc::backend::cuda