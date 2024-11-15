#pragma once
#include <type_define.h>
#include <muda/ext/linear_system.h>
#include <muda/ext/eigen/evd.h>

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

template <int N>
MUDA_DEVICE void atomic_add(muda::DenseVectorViewer<Float>& g,
                            const Vector<IndexT, N>&        indices,
                            const Vector<IndexT, N>&        is_fixed,
                            const Vector<Float, 3 * N>&     G3N)
    requires(N >= 2)
{
#pragma unroll
    for(int i = 0; i < N; ++i)
    {
        int dst = indices(i);
        if(is_fixed(i))
            continue;
        Vector3 G = G3N.template segment<3>(i * 3);
        g.template segment<3>(dst * 3).atomic_add(G);
    }
}

template <int N>
MUDA_DEVICE void atomic_add(muda::DenseVectorViewer<Float>& g,
                            const Vector<IndexT, N>&        indices,
                            const Vector<Float, 3 * N>&     G3N)
    requires(N >= 2)
{
#pragma unroll
    for(int i = 0; i < N; ++i)
    {
        int     dst = indices(i);
        Vector3 G   = G3N.template segment<3>(i * 3);
        g.template segment<3>(dst * 3).atomic_add(G);
    }
}

template <int N>
UIPC_GENERIC void assemble(muda::DoubletVectorViewer<Float, 3>& G3,
                           IndexT                               I_of_G3,
                           const Vector<IndexT, N>&             indices,
                           const Vector<IndexT, N>&             is_fixed,
                           const Vector<Float, 3 * N>&          G3N)
    requires(N >= 2)
{
    SizeT offset = I_of_G3;
#pragma unroll
    for(int i = 0; i < N; ++i)
    {
        int dst = indices(i);
        if(is_fixed(i))
            continue;
        Vector3 G = G3N.template segment<3>(i * 3);
        G3(offset++).write(dst, G);
    }
}

inline UIPC_GENERIC void assemble(muda::DoubletVectorViewer<Float, 3>& G3,
                                  IndexT                               I_of_G3,
                                  IndexT                               indices,
                                  const Vector<Float, 3>&              G3N)
{
    G3(I_of_G3).write(indices, G3N);
}

template <int N>
UIPC_GENERIC void assemble(muda::DoubletVectorViewer<Float, 3>& G3,
                           IndexT                               I_of_G3,
                           const Vector<IndexT, N>&             indices,
                           const Vector<Float, 3 * N>&          G3N)
    requires(N >= 2)
{
    SizeT offset = I_of_G3;
#pragma unroll
    for(int i = 0; i < N; ++i)
    {
        int     dst = indices(i);
        Vector3 G   = G3N.template segment<3>(i * 3);
        G3(offset++).write(dst, G);
    }
}

inline UIPC_GENERIC void assemble(muda::DoubletVectorViewer<Float, 3>& G3,
                                  IndexT                               I_of_G3,
                                  IndexT                               indices,
                                  IndexT                               is_fixed,
                                  const Vector<Float, 3>&              G3N)
{
    if(is_fixed)
        return;
    G3(I_of_G3).write(indices, G3N);
}

template <int N>
UIPC_GENERIC void assemble(muda::TripletMatrixViewer<Float, 3>& H3x3,
                           IndexT                               I_of_H3x3,
                           const Vector<IndexT, N>              indices,
                           const Vector<IndexT, N>&             is_fixed,
                           const Matrix<Float, 3 * N, 3 * N>&   H3Nx3N)
    requires(N >= 2)
{
    SizeT offset = I_of_H3x3;
#pragma unroll
    for(int i = 0; i < N; ++i)
    {
#pragma unroll
        for(int j = 0; j < N; ++j)
        {
            Matrix3x3 H = H3Nx3N.template block<3, 3>(i * 3, j * 3);

            if(is_fixed(i) || is_fixed(j))
                H.setZero();

            int L = indices(i);
            int R = indices(j);

            H3x3(offset++).write(L, R, H);
        }
    }
}

inline UIPC_GENERIC void assemble(muda::TripletMatrixViewer<Float, 3>& H3x3,
                                  IndexT                               I,
                                  IndexT                               indices,
                                  IndexT                               is_fixed,
                                  const Matrix<Float, 3, 3>&           H3Nx3N)
{
    SizeT     offset = I;
    Matrix3x3 H      = H3Nx3N;
    if(is_fixed)
        H.setZero();
    H3x3(offset).write(indices, indices, H);
}

template <int N>
UIPC_GENERIC void assemble(muda::TripletMatrixViewer<Float, 3>& H3x3,
                           IndexT                               I_of_H3x3,
                           const Vector<IndexT, N>              indices,
                           const Matrix<Float, 3 * N, 3 * N>&   H3Nx3N)
    requires(N >= 2)
{
    SizeT offset = I_of_H3x3;
#pragma unroll
    for(int i = 0; i < N; ++i)
    {
#pragma unroll
        for(int j = 0; j < N; ++j)
        {
            Matrix3x3 H = H3Nx3N.template block<3, 3>(i * 3, j * 3);

            int L = indices(i);
            int R = indices(j);

            H3x3(offset++).write(L, R, H);
        }
    }
}

inline UIPC_GENERIC void assemble(IndexT I_of_H3x3,
                                  muda::TripletMatrixViewer<Float, 3>& H3x3,
                                  IndexT                               indices,
                                  const Matrix<Float, 3, 3>&           H3Nx3N)
{
    SizeT offset = I_of_H3x3;
    H3x3(offset).write(indices, indices, H3Nx3N);
}
}  // namespace uipc::backend::cuda
