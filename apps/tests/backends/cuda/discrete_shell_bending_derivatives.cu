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

#include <app/app.h>
#include <finite_element/constitutions/discrete_shell_bending_function.h>

using namespace uipc;
using namespace uipc::backend::cuda;

TEST_CASE("discrete shell bending combined derivatives", "[cuda][discrete_shell_bending]")
{
    namespace DSB = sym::discrete_shell_bending;
    const Vector3 x0{0.0, 1.0, 0.2};
    const Vector3 x1{-1.0, 0.0, 0.0};
    const Vector3 x2{1.0, 0.0, 0.0};
    const Vector3 x3{0.0, -1.0, -0.3};

    Vector12    separate_G;
    Vector12    combined_G;
    Matrix12x12 separate_H;
    Matrix12x12 combined_H;

    DSB::dEdx(separate_G, x0, x1, x2, x3, 1.2, 0.8, 0.15, 2.0);
    DSB::ddEddx(separate_H, x0, x1, x2, x3, 1.2, 0.8, 0.15, 2.0);
    DSB::d2Edx2(combined_G, combined_H, x0, x1, x2, x3, 1.2, 0.8, 0.15, 2.0);

    CHECK((combined_G.array() == separate_G.array()).all());
    CHECK((combined_H.array() == separate_H.array()).all());
}
