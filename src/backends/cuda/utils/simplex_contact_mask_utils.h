#pragma once
#include <type_define.h>
#include <muda/viewer/dense/dense_2d.h>
namespace uipc::backend::cuda
{
inline __device__ bool allow_PT_contact(const muda::CDense2D<IndexT>& table,
                                        const Vector4i&               cids)
{
    return table(cids[0], cids[1]) && table(cids[0], cids[2]) && table(cids[0], cids[3]);
}

inline __device__ bool allow_EE_contact(const muda::CDense2D<IndexT>& table,
                                        const Vector4i&               cids)
{
    return table(cids[0], cids[2]) && table(cids[0], cids[3])
           && table(cids[1], cids[2]) && table(cids[1], cids[3]);
}

inline __device__ bool allow_PE_contact(const muda::CDense2D<IndexT>& table,
                                        const Vector3i&               cids)
{
    return table(cids[0], cids[1]) && table(cids[0], cids[2]);
}

inline __device__ bool allow_PP_contact(const muda::CDense2D<IndexT>& table,
                                        const Vector2i&               cids)
{
    return table(cids[0], cids[1]);
}
}  // namespace uipc::backend::cuda
