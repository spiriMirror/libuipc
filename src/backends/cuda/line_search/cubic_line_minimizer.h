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

#pragma once
#include <type_define.h>
#include <cmath>

namespace uipc::backend::cuda
{
/**
 * @brief Local minimizer of the cubic
 * phi(a) = E0 + slope * a + c2 * a^2 + c3 * a^3
 * that interpolates (a0, E_a0) and (a1, E_a1), a0 != a1, a0, a1 > 0
 * (Nocedal and Wright, Numerical Optimization, 2nd ed., eq. 3.59).
 *
 * The line search uses it to refine a backtracked step from the energy at
 * the start point, the energy slope g.dx along the Newton step, and the
 * energies of the last rejected and the accepted trial.
 *
 * @return The minimizer, or a negative value when phi has no local minimum
 * (including slope >= 0).
 */
inline Float cubic_line_minimizer(Float E0, Float slope, Float a0, Float E_a0, Float a1, Float E_a1)
{
    if(!(slope < 0.0) || !(a0 > 0.0) || !(a1 > 0.0) || a0 == a1)
        return -1.0;

    Float r0  = E_a0 - E0 - slope * a0;
    Float r1  = E_a1 - E0 - slope * a1;
    Float den = a0 * a0 * a1 * a1 * (a1 - a0);
    Float c3  = (a0 * a0 * r1 - a1 * a1 * r0) / den;
    Float c2  = (a1 * a1 * a1 * r0 - a0 * a0 * a0 * r1) / den;

    // phi'(a) = 3 c3 a^2 + 2 c2 a + slope; the minimizer is the root with
    // phi'' > 0. Each branch avoids cancellation.
    Float disc = c2 * c2 - 3.0 * c3 * slope;
    if(!(disc >= 0.0))
        return -1.0;
    Float root = std::sqrt(disc);
    if(c2 > 0.0)
        return -slope / (c2 + root);
    if(c3 > 0.0)
        return (root - c2) / (3.0 * c3);
    return -1.0;
}
}  // namespace uipc::backend::cuda
