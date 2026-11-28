/*
 * NVIDIA contributions: the energy-extent generation for the cached
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

#include <animator/global_animator.h>
#include <animator/animator.h>
#include <uipc/builtin/constitution_type.h>
#include <sim_engine.h>

namespace uipc::backend
{
template <>
class backend::SimSystemCreator<cuda::GlobalAnimator>
{
  public:
    static U<cuda::GlobalAnimator> create(SimEngine& engine)
    {
        auto  scene = dynamic_cast<cuda::SimEngine&>(engine).world().scene();
        auto& types = scene.constitution_tabular().types();

        // Create GlobalAnimator if there are Constraints need animator support
        bool has_constraint =
            types.find(std::string{builtin::Constraint}) != types.end();

        if(!has_constraint)
        {
            return nullptr;
        }
        return uipc::make_unique<cuda::GlobalAnimator>(engine);
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalAnimator);

void GlobalAnimator::do_build() {}

Float GlobalAnimator::substep_ratio() noexcept
{
    return m_substep_ratio;
}

SizeT GlobalAnimator::energy_extent_generation() const noexcept
{
    return m_energy_extent_generation;
}

void GlobalAnimator::init()
{
    // init frontend animator
    world().animator().init();

    // init backend animator
    for(auto&& animator : m_animators.view())
    {
        animator->init();
    }
}

void GlobalAnimator::step()
{
    // update frontend animator
    world().animator().update();

    // after frontend update, reset substep ratio
    // prepare for the next newton iteration
    m_substep_ratio = 0.0;

    // update backend animator
    for(auto&& animator : m_animators.view())
    {
        animator->step();
    }

    vector<IndexT> energy_counts;
    for(auto&& animator : m_animators.view())
    {
        auto counts = animator->energy_counts();
        energy_counts.insert(energy_counts.end(), counts.begin(), counts.end());
    }
    if(energy_counts != m_energy_counts)
    {
        m_energy_counts = std::move(energy_counts);
        ++m_energy_extent_generation;
    }
}

void GlobalAnimator::compute_substep_ratio(SizeT newton_iter)
{
    Float substep = static_cast<Float>(world().animator().substep());
    Float t       = (static_cast<Float>(newton_iter) + Float{1.0}) / substep;
    UIPC_ASSERT(substep > 0, "substep must be greater than 0");
    m_substep_ratio = std::min(t, Float{1.0});  // clamp t to [0, 1]
}

void GlobalAnimator::register_animator(Animator* animator)
{
    m_animators.register_sim_system(*animator);
}
}  // namespace uipc::backend::cuda
