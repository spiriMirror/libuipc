#include <muda/ext/eigen.h>
namespace muda
{
//using T         = float;
//constexpr int N = 3;

template <typename T, int M, int N>
void LinearSystemContext::spmv(const T&                    a,
                               CTripletMatrixView<T, M, N> A,
                               CDenseVectorView<T>         x,
                               const T&                    b,
                               DenseVectorView<T>&         y)
{
    using namespace muda;

    MUDA_ASSERT(A.extent().x == A.total_extent().x
                    && A.extent().y == A.total_extent().y
                    && A.triplet_count() == A.total_triplet_count(),
                "submatrix or subview of a Triplet Matrix is not allowed in SPMV!");

    MUDA_ASSERT(A.total_cols() * N == x.size() && A.total_rows() * M == y.size(),
                "Dimension mismatch in SPMV, A(%d, %d), x(%d), y(%d)",
                A.total_rows(),
                A.total_cols(),
                x.size(),
                y.size());

    if(b != T{0})
    {
        ParallelFor(0, stream())
            .file_line(__FILE__, __LINE__)
            .apply(y.size(),
                   [b = b, y = y.viewer().name("y")] __device__(int i) mutable
                   { y(i) = b * y(i); });
    }
    else
    {
        BufferLaunch(stream()).fill(y.buffer_view(), T{0});
    }

    ParallelFor(0, stream())
        .file_line(__FILE__, __LINE__)
        .apply(A.triplet_count(),
               [a = a,
                A = A.viewer().name("A"),
                x = x.viewer().name("x"),
                b = b,
                y = y.viewer().name("y")] __device__(int index) mutable
               {
                   auto&& [i, j, block] = A(index);
                   auto seg_x           = x.segment<N>(j * N);

                   Eigen::Vector<T, N> vec_x  = seg_x.as_eigen();
                   auto                result = a * block * vec_x;

                   auto seg_y = y.segment<M>(i * M);
                   seg_y.atomic_add(result.eval());
               });
}

template <typename T, int M, int N>
void muda::LinearSystemContext::spmv(CTripletMatrixView<T, M, N> A,
                                     CDenseVectorView<T>         x,
                                     DenseVectorView<T>          y)
{
    spmv<T, M, N>(T{1}, A, x, T{0}, y);
}
}  // namespace muda