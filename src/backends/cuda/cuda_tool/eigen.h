#pragma once
// Device-side small-matrix math used by constitutive models.
// Ported verbatim (namespace/qualifiers adjusted) from muda's ext/eigen so the
// backend keeps bit-identical numerics to the pre-migration implementation:
//   eigen::evd      — SelfAdjointEigenSolver (computeDirect for N<=3) + Jacobi variant
//   eigen::svd / pd — 3x3 SVD and polar decomposition
//   eigen::inverse  — analytic (2/3/4) + Gauss elimination (N x N)
//   eigen::atomic_add — element-wise atomicAdd for Eigen matrices
// The ::arg(std::complex) nvcc shim lives in type_define.h (included first).
#include <type_define.h>
#include <cuda_tool/eigen/evd.h>
#include <cuda_tool/eigen/svd.h>
#include <cuda_tool/eigen/inverse.h>
#include <cuda_tool/eigen/atomic.h>
