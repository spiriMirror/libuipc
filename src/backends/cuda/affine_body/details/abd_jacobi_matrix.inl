#include <muda/ext/eigen/atomic.h>
namespace uipc::backend::cuda
{
template <size_t N>
MUDA_GENERIC Vector<Float, 3 * N> ABDJacobiStack<N>::operator*(const Vector12& q) const
{
    Vector<Float, 3 * N> ret;
#pragma unroll
    for(size_t i = 0; i < N; ++i)
    {
        ret.segment<3>(3 * i) = (*m_jacobis[i]) * q;
    }
    return ret;
}

template <size_t N>
MUDA_GENERIC Matrix<Float, 3 * N, 12> ABDJacobiStack<N>::to_mat() const
{
    Matrix<Float, 3 * N, 12> ret;
    for(size_t i = 0; i < N; ++i)
    {
        ret.block<3, 12>(3 * i, 0) = m_jacobis[i]->to_mat();
    }
    return ret;
}

template <size_t N>
MUDA_GENERIC Vector12
ABDJacobiStack<N>::ABDJacobiStackT::operator*(const Vector<Float, 3 * N>& g) const
{
    Vector12 ret = Vector12::Zero();
#pragma unroll
    for(size_t i = 0; i < N; ++i)
    {
        const ABDJacobi* jacobi = m_origin.m_jacobis[i];
        ret += jacobi->T() * g.segment<3>(3 * i);
    }
    return ret;
}
}  // namespace uipc::backend::cuda