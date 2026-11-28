/*
 * NVIDIA contributions: the templated ddEddX Hessian output supporting
 * strided shared-memory workspaces and associated formatting.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Translation-invariant analytical Neo-Hookean 2D membrane shell formulas
// formulated directly in edge vectors e1 = x1 - x0, e2 = x2 - x0 with
// det(A) = ||e1 x e2||^2 to avoid O((|X|/h)^4) catastrophic cancellation
// when triangles are simulated far from the world origin.

template <typename T>
__host__ __device__ void A(Eigen::Matrix<T, 2, 2>& R, const Eigen::Vector<T, 9>& X)
{
    const Eigen::Matrix<T, 3, 1> e1(X(3) - X(0), X(4) - X(1), X(5) - X(2));
    const Eigen::Matrix<T, 3, 1> e2(X(6) - X(0), X(7) - X(1), X(8) - X(2));
    const T                      a01 = e1.dot(e2);
    R(0, 0)                          = e1.dot(e1);
    R(0, 1)                          = a01;
    R(1, 0)                          = a01;
    R(1, 1)                          = e2.dot(e2);
}

template <typename T>
__host__ __device__ void E(T&                            R,
                           const T&                      lambda,
                           const T&                      mu,
                           const Eigen::Vector<T, 9>&    X,
                           const Eigen::Matrix<T, 2, 2>& IB)
{
    using std::log;
    const Eigen::Matrix<T, 3, 1> e1(X(3) - X(0), X(4) - X(1), X(5) - X(2));
    const Eigen::Matrix<T, 3, 1> e2(X(6) - X(0), X(7) - X(1), X(8) - X(2));
    const T                      a00  = e1.dot(e1);
    const T                      a01  = e1.dot(e2);
    const T                      a11  = e2.dot(e2);
    const Eigen::Matrix<T, 3, 1> n    = e1.cross(e2);
    const T                      detA = n.dot(n);
    const T detIB  = IB(0, 0) * IB(1, 1) - IB(0, 1) * IB(1, 0);
    const T log_J2 = log(detA * detIB);
    const T tr = IB(0, 0) * a00 + (IB(0, 1) + IB(1, 0)) * a01 + IB(1, 1) * a11;

    R = T(0.125) * lambda * log_J2 * log_J2 + T(0.5) * mu * (tr - log_J2 - T(2));
}

template <typename T>
__host__ __device__ void dEdX(Eigen::Vector<T, 9>&          R,
                              const T&                      lambda,
                              const T&                      mu,
                              const Eigen::Vector<T, 9>&    X,
                              const Eigen::Matrix<T, 2, 2>& IB)
{
    using std::log;
    const Eigen::Matrix<T, 3, 1> e1(X(3) - X(0), X(4) - X(1), X(5) - X(2));
    const Eigen::Matrix<T, 3, 1> e2(X(6) - X(0), X(7) - X(1), X(8) - X(2));
    const T                      a00      = e1.dot(e1);
    const T                      a01      = e1.dot(e2);
    const T                      a11      = e2.dot(e2);
    const Eigen::Matrix<T, 3, 1> n        = e1.cross(e2);
    const T                      detA     = n.dot(n);
    const T                      inv_detA = T(1) / detA;
    const T                      invA00   = a11 * inv_detA;
    const T                      invA01   = -a01 * inv_detA;
    const T                      invA11   = a00 * inv_detA;
    const T detIB  = IB(0, 0) * IB(1, 1) - IB(0, 1) * IB(1, 0);
    const T log_J2 = log(detA * detIB);
    const T alpha  = T(0.5) * lambda * log_J2 - mu;
    const T ib01   = T(0.5) * (IB(0, 1) + IB(1, 0));
    const T s00    = mu * IB(0, 0) + alpha * invA00;
    const T s01    = mu * ib01 + alpha * invA01;
    const T s11    = mu * IB(1, 1) + alpha * invA11;

    const Eigen::Matrix<T, 3, 1> g1 = s00 * e1 + s01 * e2;
    const Eigen::Matrix<T, 3, 1> g2 = s01 * e1 + s11 * e2;
    const Eigen::Matrix<T, 3, 1> g0 = -(g1 + g2);

    R(0) = g0(0);
    R(1) = g0(1);
    R(2) = g0(2);
    R(3) = g1(0);
    R(4) = g1(1);
    R(5) = g1(2);
    R(6) = g2(0);
    R(7) = g2(1);
    R(8) = g2(2);
}

template <typename T, typename Hessian = Eigen::Matrix<T, 9, 9>>
__host__ __device__ void ddEddX(Hessian&                      R,
                                const T&                      lambda,
                                const T&                      mu,
                                const Eigen::Vector<T, 9>&    X,
                                const Eigen::Matrix<T, 2, 2>& IB)
{
    using std::log;
    const Eigen::Matrix<T, 3, 1> e1(X(3) - X(0), X(4) - X(1), X(5) - X(2));
    const Eigen::Matrix<T, 3, 1> e2(X(6) - X(0), X(7) - X(1), X(8) - X(2));
    const T                      a00      = e1.dot(e1);
    const T                      a01      = e1.dot(e2);
    const T                      a11      = e2.dot(e2);
    const Eigen::Matrix<T, 3, 1> n        = e1.cross(e2);
    const T                      detA     = n.dot(n);
    const T                      inv_detA = T(1) / detA;
    const T                      invA00   = a11 * inv_detA;
    const T                      invA01   = -a01 * inv_detA;
    const T                      invA11   = a00 * inv_detA;
    const T detIB  = IB(0, 0) * IB(1, 1) - IB(0, 1) * IB(1, 0);
    const T log_J2 = log(detA * detIB);
    const T alpha  = T(0.5) * lambda * log_J2 - mu;
    const T ib01   = T(0.5) * (IB(0, 1) + IB(1, 0));
    const T s00    = mu * IB(0, 0) + alpha * invA00;
    const T s01    = mu * ib01 + alpha * invA01;
    const T s11    = mu * IB(1, 1) + alpha * invA11;

    // Q = [q0, q1] = D * A^{-1}, P = D * A^{-1} * D^T (in-plane projector)
    const Eigen::Matrix<T, 3, 1> q0 = invA00 * e1 + invA01 * e2;
    const Eigen::Matrix<T, 3, 1> q1 = invA01 * e1 + invA11 * e2;
    const Eigen::Matrix<T, 3, 3> P  = e1 * q0.transpose() + e2 * q1.transpose();

    Eigen::Matrix<T, 3, 3> H11 =
        (lambda - alpha) * (q0 * q0.transpose()) - (alpha * invA00) * P;
    H11(0, 0) += s00;
    H11(1, 1) += s00;
    H11(2, 2) += s00;

    Eigen::Matrix<T, 3, 3> H12 =
        lambda * (q0 * q1.transpose()) - alpha * (invA01 * P + q1 * q0.transpose());
    H12(0, 0) += s01;
    H12(1, 1) += s01;
    H12(2, 2) += s01;

    Eigen::Matrix<T, 3, 3> H22 =
        (lambda - alpha) * (q1 * q1.transpose()) - (alpha * invA11) * P;
    H22(0, 0) += s11;
    H22(1, 1) += s11;
    H22(2, 2) += s11;

    const Eigen::Matrix<T, 3, 3> H21 = H12.transpose();
    const Eigen::Matrix<T, 3, 3> H01 = -(H11 + H21);
    const Eigen::Matrix<T, 3, 3> H02 = -(H12 + H22);
    const Eigen::Matrix<T, 3, 3> H10 = H01.transpose();
    const Eigen::Matrix<T, 3, 3> H20 = H02.transpose();
    const Eigen::Matrix<T, 3, 3> H00 = -(H01 + H02);

    auto write_block = [&](int br, int bc, const Eigen::Matrix<T, 3, 3>& M)
    {
        for(int r = 0; r < 3; ++r)
            for(int c = 0; c < 3; ++c)
                R(3 * br + r, 3 * bc + c) = M(r, c);
    };
    write_block(0, 0, H00);
    write_block(0, 1, H01);
    write_block(0, 2, H02);
    write_block(1, 0, H10);
    write_block(1, 1, H11);
    write_block(1, 2, H12);
    write_block(2, 0, H20);
    write_block(2, 1, H21);
    write_block(2, 2, H22);
}
