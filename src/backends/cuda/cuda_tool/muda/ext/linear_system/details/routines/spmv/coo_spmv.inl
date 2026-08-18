namespace muda
{
template <typename T>
void LinearSystemContext::spmv(const T&            a,
                               CCOOMatrixView<T>   A,
                               CDenseVectorView<T> x,
                               const T&            b,
                               DenseVectorView<T>& y)
{
    auto op = A.is_trans() ? CUSPARSE_OPERATION_TRANSPOSE : CUSPARSE_OPERATION_NON_TRANSPOSE;
    generic_spmv(a, op, A.descr(), x.descr(), b, y.descr());
}

template <typename T>
void LinearSystemContext::spmv(CCOOMatrixView<T> A, CDenseVectorView<T> x, DenseVectorView<T> y)
{
    spmv(T{1}, A, x, T{0}, y);
}
}  // namespace muda