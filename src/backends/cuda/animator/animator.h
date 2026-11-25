/*
 * NVIDIA contributions: per-constraint energy-count interface for the cached
 * line-search energy graph.
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
#include <animator/global_animator.h>

namespace uipc::backend::cuda
{
class Animator : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

  private:
    friend class GlobalAnimator;
    void               init();
    void               step();
    span<const IndexT> energy_counts() const noexcept;
    virtual void       do_build() override final;

    GlobalAnimator* m_global_animator = nullptr;

  protected:
    Float        substep_ratio() const noexcept;
    virtual void do_init()                 = 0;
    virtual void do_step()                 = 0;
    virtual void do_build(BuildInfo& info) = 0;
    // Line-search energy count of each constraint after the last step()
    // (empty: the animator has no line-search energy).
    virtual span<const IndexT> do_energy_counts() const noexcept { return {}; }
};
}  // namespace uipc::backend::cuda
