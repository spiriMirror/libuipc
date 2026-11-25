/*
 * NVIDIA contributions: launch/finish interfaces of the tolerance checks.
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

#pragma once
#include <sim_system.h>
#include <newton_tolerance/newton_tolerance_manager.h>

namespace uipc::backend::cuda
{
class NewtonToleranceChecker : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

    class InitInfo
    {
      public:
    };

    using CheckResultInfo = NewtonToleranceManager::CheckResultInfo;
    using PreNewtonInfo   = NewtonToleranceManager::PreNewtonInfo;
    using LaunchCheckInfo = NewtonToleranceManager::LaunchCheckInfo;

    class Impl
    {
      public:
    };

  protected:
    virtual void do_build(BuildInfo& info)          = 0;
    virtual void do_init(InitInfo& info)            = 0;
    virtual void do_pre_newton(PreNewtonInfo& info) = 0;
    virtual void do_check(CheckResultInfo& info)    = 0;
    // Optional first half of do_check(): enqueue the device work and the
    // copy of its result to the host on info.stream(). do_check() then sees
    // CheckResultInfo::launched() and only reads the result. A checker
    // without it does all of its work in do_check().
    virtual void        do_launch_check(LaunchCheckInfo& info);
    virtual std::string do_report();

  private:
    friend class NewtonToleranceManager;
    virtual void do_build() override;
    void         init();  // only called by NewtonToleranceManager

    void        pre_newton(PreNewtonInfo& info);
    void        launch_check(LaunchCheckInfo& info);
    void        check(CheckResultInfo& info);
    std::string report();
    Impl        m_impl;
};
}  // namespace uipc::backend::cuda