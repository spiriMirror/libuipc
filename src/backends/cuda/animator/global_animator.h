/*
 * NVIDIA contributions: the energy-extent generation interface.
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

namespace uipc::backend::cuda
{
class Animator;
class GlobalAnimator final : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    /**
     * @brief aim_pos_this_iter = aim_position * alpha + prev_position * (1 - alpha)
     */
    Float substep_ratio() noexcept;
    // Changes whenever step() changes the line-search energy count of a
    // constraint, i.e. the extent of an energy producer's launches and the
    // energy ranges the producers write (LineSearcher keys its cached
    // energy graph on it).
    SizeT energy_extent_generation() const noexcept;

  private:
    friend class SimEngine;
    void init();  // only be called by SimEngine
    void step();  // only be called by SimEngine
    void compute_substep_ratio(SizeT newton_iter);  // only be called by SimEngine
    friend class Animator;
    void register_animator(Animator* animator);  // only be called by Animator


    virtual void do_build() override;

    SimSystemSlotCollection<Animator> m_animators;

    Float m_substep_ratio = 1.0;
    vector<IndexT> m_energy_counts;  // of every animator's constraints, after step()
    SizeT m_energy_extent_generation = 0;
};
}  // namespace uipc::backend::cuda
