/*
 * NVIDIA contributions: the launch/finish split of the tolerance checks.
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

#include <newton_tolerance/newton_tolerance_checker.h>

namespace uipc::backend::cuda
{
std::string NewtonToleranceChecker::do_report()
{
    return "";
}

void NewtonToleranceChecker::do_launch_check(LaunchCheckInfo& info) {}
void NewtonToleranceChecker::do_build()
{
    auto& manager = require<NewtonToleranceManager>();

    BuildInfo info;
    do_build(info);  // let derived classes build their specific info

    manager.add_checker(this);  // Register this checker with the manager
}

void NewtonToleranceChecker::init()
{
    InitInfo info;
    do_init(info);  // let derived classes initialize their specific info
}

void NewtonToleranceChecker::pre_newton(PreNewtonInfo& info)
{
    PreNewtonInfo pre_info;
    do_pre_newton(pre_info);  // let derived classes prepare for newton step
}

void NewtonToleranceChecker::launch_check(LaunchCheckInfo& info)
{
    do_launch_check(info);
}

void NewtonToleranceChecker::check(CheckResultInfo& info)
{
    do_check(info);
}

std::string NewtonToleranceChecker::report()
{
    return do_report();
}
}  // namespace uipc::backend::cuda
