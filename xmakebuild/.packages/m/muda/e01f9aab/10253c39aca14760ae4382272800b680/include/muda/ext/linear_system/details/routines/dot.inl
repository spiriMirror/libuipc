namespace muda
{
namespace details::linear_system
{
    template <typename T>
    MUDA_INLINE void dot_common_check(CDenseVectorView<T> x, CDenseVectorView<T> y)
    {
        MUDA_ASSERT(x.data() && y.data(), "x.data() and y.data() should not be nullptr");
        MUDA_ASSERT(x.size() / x.inc() == y.size() / y.inc(),
                    "x (size=%lld, inc=%d) should be the same as y (size=%lld, inc=%d)",
                    x.size(),
                    x.inc(),
                    y.size(),
                    y.inc());
    }
}  // namespace details::linear_system


template <typename T>
void LinearSystemContext::dot(CDenseVectorView<T> x, CDenseVectorView<T> y, T* result)
{
    set_pointer_mode_host();
    details::linear_system::dot_common_check(x, y);

    auto type = cuda_data_type<T>();
    auto size = x.size() / x.inc();

    checkCudaErrors(cublasDotEx(
        cublas(), size, x.data(), type, x.inc(), y.data(), type, y.inc(), result, type, type));
}

template <typename T>
T LinearSystemContext::dot(CDenseVectorView<T> x, CDenseVectorView<T> y)
{
    T result;
    dot(x, y, &result);
    sync();
    return result;
}

template <typename T>
void LinearSystemContext::dot(CDenseVectorView<T> x, CDenseVectorView<T> y, VarView<T> result)
{
    set_pointer_mode_device();
    details::linear_system::dot_common_check(x, y);

    auto type = cuda_data_type<T>();
    auto size = x.size() / x.inc();


    checkCudaErrors(cublasDotEx(
        cublas(), size, x.data(), type, x.inc(), y.data(), type, y.inc(), result.data(), type, type));
}

}  // namespace muda