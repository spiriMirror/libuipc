/*
 * NVIDIA contributions: per-constraint energy counts for the cached
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

#include <animator/animator.h>

namespace uipc::backend::cuda
{
void Animator::do_build()
{
    m_global_animator = &require<GlobalAnimator>();

    BuildInfo info;
    do_build(info);

    m_global_animator->register_animator(this);
}

Float Animator::substep_ratio() const noexcept
{
    return m_global_animator->substep_ratio();
}

void Animator::init()
{
    do_init();
}

void Animator::step()
{
    do_step();
}

span<const IndexT> Animator::energy_counts() const noexcept
{
    return do_energy_counts();
}
}  // namespace uipc::backend::cuda
