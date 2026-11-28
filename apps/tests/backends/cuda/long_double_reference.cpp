/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "long_double_reference.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <limits>

namespace uipc::test_reference
{
namespace
{
    template <int N>
    void psd_projection(const std::vector<double>& input, int count, std::vector<long double>& out)
    {
        using MatrixL = Eigen::Matrix<long double, N, N>;
        for(int I = 0; I < count; ++I)
        {
            MatrixL M;
            for(int e = 0; e < N * N; ++e)
                M(e % N, e / N) = static_cast<long double>(input[I * N * N + e]);
            Eigen::SelfAdjointEigenSolver<MatrixL> es(M);
            const MatrixL                          P = es.eigenvectors()
                              * es.eigenvalues().cwiseMax(0.0L).asDiagonal()
                              * es.eigenvectors().transpose();
            for(int e = 0; e < N * N; ++e)
                out[I * N * N + e] = P(e % N, e / N);
        }
    }
}  // namespace

std::vector<long double> psd_projection_long_double(int N, const std::vector<double>& input, int count)
{
    std::vector<long double> out(static_cast<size_t>(count) * N * N);
    if(N == 9)
        psd_projection<9>(input, count, out);
    else if(N == 12)
        psd_projection<12>(input, count, out);
    return out;
}

double max_error_long_double(const std::vector<double>&      out,
                             const std::vector<long double>& reference,
                             int                             offset,
                             int                             count,
                             double                          scale)
{
    long double e = 0;
    for(int k = 0; k < count; ++k)
    {
        const long double d =
            static_cast<long double>(out[offset + k]) - reference[offset + k];
        e = std::max(e, d < 0 ? -d : d);
    }
    return static_cast<double>(e / static_cast<long double>(scale));
}

bool long_double_is_extended()
{
    return std::numeric_limits<long double>::digits > std::numeric_limits<double>::digits;
}
}  // namespace uipc::test_reference
