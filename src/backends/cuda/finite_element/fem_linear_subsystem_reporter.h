/*
 * NVIDIA contributions: the beside-contact launch interface.
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
#include <finite_element/fem_linear_subsystem.h>

namespace uipc::backend::cuda
{
class FEMLinearSubsystemReporter : public SimSystem
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

    using ReportExtentInfo = FEMLinearSubsystem::ReportExtentInfo;
    using AssembleInfo     = FEMLinearSubsystem::AssembleInfo;

  protected:
    virtual void do_build(BuildInfo& info)                = 0;
    virtual void do_init(InitInfo& info)                  = 0;
    virtual void do_report_extent(ReportExtentInfo& info) = 0;
    virtual void do_assemble(AssembleInfo& info)          = 0;
    // True for a reporter whose producers are light enough to run beside the
    // contact assembly kernels (no block that takes a whole SM): a prelaunch
    // starts them before the contact join, the others after it.
    virtual bool do_launches_beside_contact() const noexcept { return false; }

  private:
    friend class FEMLinearSubsystem;
    virtual void do_build() final override;
    void         init();
    void         report_extent(ReportExtentInfo& info);
    void         assemble(AssembleInfo& info);
    bool         launches_beside_contact() const noexcept
    {
        return do_launches_beside_contact();
    }

    IndexT m_index = -1;
};
}  // namespace uipc::backend::cuda
