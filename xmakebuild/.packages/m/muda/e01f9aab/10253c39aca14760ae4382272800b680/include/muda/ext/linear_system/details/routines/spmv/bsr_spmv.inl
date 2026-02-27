namespace muda
{
//using T         = double;
//constexpr int N = 4;
namespace detail::linear_system
{
    template <typename T>
    void bsrmv(cusparseHandle_t          handle,
               size_t                    block_rows,
               size_t                    block_cols,
               size_t                    non_zeros,
               const T*                  a,
               cusparseOperation_t       op,
               const cusparseMatDescr_t& descrA,
               const T*                  val_A,
               const int*                block_row_offsets,
               const int*                block_col_indices,
               size_t                    N,
               const T*                  x,
               const T*                  b,
               T*                        y)
    {
        if constexpr(std::is_same_v<T, float>)
        {
            checkCudaErrors(cusparseSbsrmv(handle,
                                           CUSPARSE_DIRECTION_COLUMN,
                                           op,
                                           block_rows,
                                           block_cols,
                                           non_zeros,
                                           a,
                                           descrA,
                                           val_A,
                                           block_row_offsets,
                                           block_col_indices,
                                           N,
                                           x,
                                           b,
                                           y));
        }
        else if constexpr(std::is_same_v<T, double>)
        {
            checkCudaErrors(cusparseDbsrmv(handle,
                                           CUSPARSE_DIRECTION_COLUMN,
                                           op,
                                           block_rows,
                                           block_cols,
                                           non_zeros,
                                           a,
                                           descrA,
                                           val_A,
                                           block_row_offsets,
                                           block_col_indices,
                                           N,
                                           x,
                                           b,
                                           y));
        }
        else
        {
            static_assert(always_false_v<T>, "T must be float or double");
        }
    }
}  // namespace detail::linear_system

template <typename T, int N>
void LinearSystemContext::spmv(const T&             a,
                               CBSRMatrixView<T, N> A,
                               CDenseVectorView<T>  x,
                               const T&             b,
                               DenseVectorView<T>&  y)
{
    set_pointer_mode_host();

    auto op = A.is_trans() ? CUSPARSE_OPERATION_TRANSPOSE : CUSPARSE_OPERATION_NON_TRANSPOSE;

    detail::linear_system::bsrmv<T>(cusparse(),
                                    A.rows(),
                                    A.cols(),
                                    A.non_zeros(),
                                    &a,
                                    op,
                                    A.legacy_descr(),
                                    (const T*)A.values(),
                                    A.row_offsets(),
                                    A.col_indices(),
                                    N,
                                    x.data(),
                                    &b,
                                    y.data());
}
template <typename T, int N>
void muda::LinearSystemContext::spmv(CBSRMatrixView<T, N> A,
                                     CDenseVectorView<T>  x,
                                     DenseVectorView<T>   y)
{
    spmv(T{1}, A, x, T{0}, y);
}
}  // namespace muda