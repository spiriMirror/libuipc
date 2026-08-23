#pragma once
#include <cuda_tool/cuda_tool.h>
#include <cuda_tool/eigen/evd.h>
#include <algorithm/qr_svd.hpp>

namespace uipc::backend::cuda
{
namespace sym::stable_neo_hookean_3d
{
#include "detail/stable_neo_hookean_3d.inl"
}

// Stiff-GIPC's SNK1: energy + gradient + analytically SPD-projected Hessian
// (replaces the SymEigen-generated derivatives + generic make_spd EVD)
namespace snk1
{
#include "detail/stable_neo_hookean_3d_snk1.inl"
}
}  // namespace uipc::backend::cuda
