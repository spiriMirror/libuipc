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

namespace uipc::backend::cuda
{
// alpha * dx rounded once per component and never fused into an FMA.
// step_forward (x = x0 + alpha * dx) and the trajectory broad phase (swept box
// x0 .. x0 + alpha * dx) both use it, so the accepted position of a line search
// with alpha_acc == alpha is bit-identical to the swept-box end point, and a
// smaller alpha_acc stays inside the box because the rounding is monotone.
// This lets Newton iteration k > 0 reuse the trajectory candidates of k - 1.
inline UIPC_GENERIC Vector3 step_displacement(Float alpha, const Vector3& dx)
{
#ifdef __CUDA_ARCH__
    return Vector3{__dmul_rn(alpha, dx.x()),
                   __dmul_rn(alpha, dx.y()),
                   __dmul_rn(alpha, dx.z())};
#else
    return alpha * dx;
#endif
}

inline UIPC_GENERIC Vector3 step_position(const Vector3& x0, Float alpha, const Vector3& dx)
{
    const Vector3 d = step_displacement(alpha, dx);
#ifdef __CUDA_ARCH__
    return Vector3{__dadd_rn(x0.x(), d.x()),
                   __dadd_rn(x0.y(), d.y()),
                   __dadd_rn(x0.z(), d.z())};
#else
    return x0 + d;
#endif
}
}  // namespace uipc::backend::cuda
