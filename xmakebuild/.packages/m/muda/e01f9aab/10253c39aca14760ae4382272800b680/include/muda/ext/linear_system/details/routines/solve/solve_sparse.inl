#include <muda/check/check_cusolver.h>
namespace muda
{
// using T = float;
namespace details::linear_system
{
    template <typename T>
    void svqr(cusolverSpHandle_t       handle,
              int                      m,
              int                      nnz,
              const cusparseMatDescr_t descrA,
              const T*                 csrValA,
              const int*               csrRowPtrA,
              const int*               csrColIndA,
              const T*                 b,
              T                        tol,
              int                      reorder,
              T*                       x,
              int*                     singularity)

    {
        if constexpr(std::is_same_v<T, float>)
        {
            checkCudaErrors(cusolverSpScsrlsvqr(
                handle, m, nnz, descrA, csrValA, csrRowPtrA, csrColIndA, b, tol, reorder, x, singularity));
        }
        else if constexpr(std::is_same_v<T, double>)
        {
            checkCudaErrors(cusolverSpDcsrlsvqr(
                handle, m, nnz, descrA, csrValA, csrRowPtrA, csrColIndA, b, tol, reorder, x, singularity));
        }
        else
        {
            static_assert(always_false_v<T>, "Unsupported type");
        }
    }
}  // namespace details::linear_system


template <typename T>
void LinearSystemContext::solve(DenseVectorView<T> x, CCSRMatrixView<T> A, CDenseVectorView<T> b)
{
    MUDA_ASSERT(!A.is_trans(), "CSRMatrix A must not be transposed");

    auto handle = cusolver_sp();

    auto singularity = std::make_shared<int>(0);

    details::linear_system::svqr(handle,
                                 A.rows(),
                                 A.non_zeros(),
                                 A.legacy_descr(),
                                 A.values(),
                                 A.row_offsets(),
                                 A.col_indices(),
                                 b.data(),
                                 m_tolerance.solve_sparse_error_threshold<T>(),
                                 m_reorder.reorder_method_int(),
                                 x.data(),
                                 singularity.get());

    std::string label{this->label()};
    this->label("");  // remove label because we consume it here

    add_sync_callback(
        [info = std::move(singularity), label = std::move(label)]() mutable
        {
            int result = *info;
            if(result != -1)
            {
                MUDA_KERNEL_WARN_WITH_LOCATION("In calling label %s: A*x=b solving failed. R(%d,%d) is almost 0",
                                               label.c_str(),
                                               result,
                                               result);
            }
        });
}
}  // namespace muda