/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
// Host-only 80-bit (long double) references for the backend accuracy tests.
// Implemented in long_double_reference.cpp, a pure host translation unit:
// nvcc compiles Eigen's long double code for the device as well and rejects it.
#include <vector>

namespace uipc::test_reference
{
// P = V max(L, 0) V^T of `count` symmetric N x N matrices (column-major
// doubles), each computed with Eigen's SelfAdjointEigenSolver in long double.
std::vector<long double> psd_projection_long_double(int N,
                                                    const std::vector<double>& input,
                                                    int count);

// max_k |out[offset + k] - reference[offset + k]| / scale, k < count, in
// long double.
double max_error_long_double(const std::vector<double>&      out,
                             const std::vector<long double>& reference,
                             int                             offset,
                             int                             count,
                             double                          scale);

// Whether long double is wider than double (it is not with MSVC); only then is
// the reference more precise than the double computations it checks.
bool long_double_is_extended();
}  // namespace uipc::test_reference
