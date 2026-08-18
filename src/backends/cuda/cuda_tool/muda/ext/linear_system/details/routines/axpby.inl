namespace muda
{
namespace details::linear_system
{
    template <typename T>
    void axpby_common_check(CDenseVectorView<T> x, DenseVectorView<T> y)
    {
        MUDA_ASSERT(x.data(), "Vector x is empty");
        MUDA_ASSERT(y.data(), "Vector y is empty");
        MUDA_ASSERT(x.size() / x.inc() == y.size() / y.inc(),
                    "Vector x and y have different size, x (size=%lld, inc=%d), y (size=%lld, inc=%d)",
                    x.size(),
                    x.inc(),
                    y.size(),
                    y.inc());
    }
    template <typename T>
    void axpby_common_check(CDenseVectorView<T> x, CDenseVectorView<T> y, DenseVectorView<T> z)
    {
        MUDA_ASSERT(x.data(), "Vector x is empty");
        MUDA_ASSERT(y.data(), "Vector y is empty");
        MUDA_ASSERT(z.data(), "Vector z is empty");
        MUDA_ASSERT(x.size() / x.inc() == y.size() / y.inc(),
                    "Vector x and y have different size, x (size=%lld, inc=%d), y (size=%lld, inc=%d)",
                    x.size(),
                    x.inc(),
                    y.size(),
                    y.inc());
        MUDA_ASSERT(x.size() / x.inc() == z.size() / z.inc(),
                    "Vector x and z have different size, x (size=%lld, inc=%d), z (size=%lld, inc=%d)",
                    x.size(),
                    x.inc(),
                    z.size(),
                    z.inc());
    }
}  // namespace details::linear_system
template <typename T>
void LinearSystemContext::axpby(const T&            alpha,
                                CDenseVectorView<T> x,
                                const T&            beta,
                                DenseVectorView<T>  y)
{
    details::linear_system::axpby_common_check(x, y);
    auto size = x.size() / x.inc();
    ParallelFor().apply(size,
                        [x     = x.buffer_view(),
                         x_inc = x.inc(),
                         y     = y.buffer_view(),
                         y_inc = y.inc(),
                         a     = alpha,
                         b     = beta] __device__(int i) mutable
                        {
                            auto& r_y = *y.data(i * y_inc);
                            auto& r_x = *x.data(i * x_inc);
                            r_y       = a * r_x + b * r_y;
                        });
}
template <typename T>
void muda::LinearSystemContext::axpby(CVarView<T>         alpha,
                                      CDenseVectorView<T> x,
                                      CVarView<T>         beta,
                                      DenseVectorView<T>  y)
{
    details::linear_system::axpby_common_check(x, y);
    auto size = x.size() / x.inc();
    ParallelFor().apply(size,
                        [x     = x.buffer_view(),
                         x_inc = x.inc(),
                         y     = y.buffer_view(),
                         y_inc = y.inc(),
                         a     = alpha.data(),
                         b     = beta.data()] __device__(int i) mutable
                        {
                            auto& r_y = *y.data(i * y_inc);
                            auto& r_x = *x.data(i * x_inc);
                            r_y       = *a * r_x + *b * r_y;
                        });
}
template <typename T>
void LinearSystemContext::plus(CDenseVectorView<T> x, CDenseVectorView<T> y, DenseVectorView<T> z)
{
    details::linear_system::axpby_common_check(x, y, z);
    auto size = x.size() / x.inc();
    ParallelFor().apply(size,
                        [x     = x.buffer_view(),
                         x_inc = x.inc(),
                         y     = y.buffer_view(),
                         y_inc = y.inc(),
                         z     = z.buffer_view(),
                         z_inc = z.inc()] __device__(int i) mutable
                        {
                            auto& r_z = *z.data(i * z_inc);
                            auto& r_x = *x.data(i * x_inc);
                            auto& r_y = *y.data(i * y_inc);
                            r_z       = r_x + r_y;
                        });
}
}  // namespace muda