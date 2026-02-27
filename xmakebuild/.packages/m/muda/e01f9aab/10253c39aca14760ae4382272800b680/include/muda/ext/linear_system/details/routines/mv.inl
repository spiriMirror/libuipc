namespace muda
{
namespace details::linear_system
{
    template <typename T>
    MUDA_INLINE void mv_common_check(CDenseMatrixView<T> A,
                                     CDenseVectorView<T> x,
                                     DenseVectorView<T>  y)
    {
        MUDA_ASSERT(A.col() == x.size() && A.row() == y.size(),
                    "Dimension mismatch in MV, A -> (%d, %d), x -> (%d), y -> (%d)",
                    A.row(),
                    A.col(),
                    x.size(),
                    y.size());
        MUDA_ASSERT(A.data(), "Matrix A is empty");
        MUDA_ASSERT(x.data(), "Vector x is empty");
        MUDA_ASSERT(y.data(), "Vector y is empty");
    }

    template <typename T>
    void gemv(cublasHandle_t    handle,
              cublasOperation_t trans,
              int64_t           m,
              int64_t           n,
              const T*          alpha,
              const T*          A,
              int64_t           lda,
              const T*          x,
              int64_t           incx,
              const T*          beta,
              T*                y,
              int64_t           incy)
    {
        if constexpr(std::is_same_v<T, float>)
        {
            checkCudaErrors(cublasSgemv_v2_64(
                handle, trans, m, n, alpha, A, lda, x, incx, beta, y, incy));
        }
        else if constexpr(std::is_same_v<T, double>)
        {
            checkCudaErrors(cublasDgemv_v2_64(
                handle, trans, m, n, alpha, A, lda, x, incx, beta, y, incy));
        }
    }

    template <typename T>
    void symv(cublasHandle_t handle,
              int64_t        m,
              const T*       alpha,
              const T*       A,
              int64_t        lda,
              const T*       x,
              int64_t        incx,
              const T*       beta,
              T*             y,
              int64_t        incy)
    {
        if constexpr(std::is_same_v<T, float>)
        {
            checkCudaErrors(cublasSsymv_v2_64(
                handle, cublasFillMode_t::CUBLAS_FILL_MODE_LOWER, m, alpha, A, lda, x, incx, beta, y, incy));
        }
        else if constexpr(std::is_same_v<T, double>)
        {
            checkCudaErrors(cublasDsymv_v2_64(
                handle, cublasFillMode_t::CUBLAS_FILL_MODE_LOWER, m, alpha, A, lda, x, incx, beta, y, incy));
        }
    }
}  // namespace details::linear_system
template <typename T>
void LinearSystemContext::mv(CDenseMatrixView<T> A,
                             const T&            alpha,
                             CDenseVectorView<T> x,
                             const T&            beta,
                             DenseVectorView<T>  y)
{
    set_pointer_mode_host();
    details::linear_system::mv_common_check(A, x, y);

    if(A.is_sym())
    {
        MUDA_ASSERT(A.row() == A.col(), "A must be square matrix");

        details::linear_system::symv<T>(cublas(),
                                        A.row(),
                                        &alpha,
                                        A.data(),
                                        A.lda(),
                                        x.data(),
                                        (int64_t)x.inc(),
                                        &beta,
                                        y.data(),
                                        (int64_t)y.inc());
    }
    else
    {
        details::linear_system::gemv<T>(cublas(),
                                        cublas_trans_operation(A.is_trans()),
                                        A.row(),
                                        A.col(),
                                        &alpha,
                                        A.data(),
                                        A.lda(),
                                        x.data(),
                                        (int64_t)x.inc(),
                                        &beta,
                                        y.data(),
                                        (int64_t)y.inc());
    }
}

template <typename T>
void LinearSystemContext::mv(CDenseMatrixView<T> A,
                             CVarView<T>         alpha,
                             CDenseVectorView<T> x,
                             CVarView<T>         beta,
                             DenseVectorView<T>  y)
{
    set_pointer_mode_device();
    details::linear_system::mv_common_check(A, x, y);

    if(A.is_sym())
    {
        MUDA_ASSERT(A.row() == A.col(), "A must be square matrix");

        details::linear_system::symv<T>(cublas(),
                                        A.row(),
                                        alpha.data(),
                                        A.data(),
                                        A.lda(),
                                        x.data(),
                                        (int64_t)x.inc(),
                                        beta.data(),
                                        y.data(),
                                        (int64_t)y.inc());
    }
    else
    {
        details::linear_system::gemv<T>(cublas(),
                                        cublas_trans_operation(A.is_trans()),
                                        A.row(),
                                        A.col(),
                                        alpha.data(),
                                        A.data(),
                                        A.lda(),
                                        x.data(),
                                        (int64_t)x.inc(),
                                        beta.data(),
                                        y.data(),
                                        (int64_t)y.inc());
    }
}

template <typename T>
void LinearSystemContext::mv(CDenseMatrixView<T> A, CDenseVectorView<T> x, DenseVectorView<T> y)
{
    mv(A, T(1), x, T(0), y);
}
}  // namespace muda