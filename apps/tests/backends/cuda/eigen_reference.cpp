/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "eigen_reference.h"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace uipc::test_reference
{
bool symmetric_eigen(int N, const double* matrix, double* eigenvalues, double* eigenvectors)
{
    const Eigen::Map<const Eigen::MatrixXd>              M(matrix, N, N);
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(M);
    if(solver.info() != Eigen::Success)
        return false;
    Eigen::Map<Eigen::VectorXd>(eigenvalues, N) = solver.eigenvalues();
    if(eigenvectors)
        Eigen::Map<Eigen::MatrixXd>(eigenvectors, N, N) = solver.eigenvectors();
    return true;
}

void householder_q(int N, const double* matrix, double* q)
{
    const Eigen::HouseholderQR<Eigen::MatrixXd> qr(
        Eigen::Map<const Eigen::MatrixXd>(matrix, N, N));
    Eigen::Map<Eigen::MatrixXd>(q, N, N) = qr.householderQ();
}
}  // namespace uipc::test_reference
