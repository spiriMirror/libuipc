#pragma once
#include <type_define.h>
#include <muda/ext/eigen/evd.h>

namespace uipc::backend::cuda
{
template <int N>
UIPC_GENERIC void make_spd(Matrix<Float, N, N>& H)
{
    Vector<Float, N>    eigen_values;
    Matrix<Float, N, N> eigen_vectors;
    muda::eigen::template evd<Float, N>(H, eigen_values, eigen_vectors);
    for(int i = 0; i < N; ++i)
    {
        auto& v = eigen_values(i);
        v       = v < 0.0 ? 0.0 : v;
    }
    H = eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.transpose();
}
}