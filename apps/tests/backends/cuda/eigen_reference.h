/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
// Host-only Eigen decompositions for the backend tests' CPU references and
// random inputs. They are compiled once in eigen_reference.cpp, a plain C++
// translation unit, so the .cu tests do not pass Eigen's solvers through nvcc.

namespace uipc::test_reference
{
// Eigen's SelfAdjointEigenSolver on the symmetric N x N matrix `matrix`
// (column-major). Returns whether it succeeded; it then writes the eigenvalues
// in ascending order and, unless `eigenvectors` is null, the eigenvectors as
// the columns of a column-major N x N matrix.
bool symmetric_eigen(int N, const double* matrix, double* eigenvalues, double* eigenvectors = nullptr);

// The orthogonal factor Q of Eigen's HouseholderQR of the N x N matrix `matrix`
// (column-major), written to `q` (column-major N x N).
void householder_q(int N, const double* matrix, double* q);
}  // namespace uipc::test_reference
