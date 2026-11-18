/*
 * NVIDIA contributions: tolerance-check work launched on the line-search
 * prologue stream and the read-back of its residual through mapped memory.
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
#include <global_geometry/global_vertex_manager.h>
#include <uipc/geometry/attribute_slot.h>

namespace uipc::backend::cuda
{
class MaxTranslationChecker : public NewtonToleranceChecker
{
  public:
    using NewtonToleranceChecker::NewtonToleranceChecker;

    SimSystemSlot<GlobalVertexManager>      vertex_manager;
    S<const geometry::AttributeSlot<Float>> dt_attr;
    Float                                   newton_velocity_tol   = 0.0;
    Float                                   velocity_tol_relative = 0.0;
    Float                                   res0                  = 0.0;
    Float                                   res                   = 0.0;
    Float                                   rel_tol               = 0.0;
    Float                                   abs_tol               = 0.0;
    SizeT                                   frame                 = 0;
    // stored by the launched reduction (no copy follows it on the mapped path)
    cuda_tool::ReadbackBuffer<Float> launched_res;

    void do_build(BuildInfo& info) override
    {
        vertex_manager = require<GlobalVertexManager>();
        auto& config   = world().scene().config();
        dt_attr        = config.find<Float>("dt");
        UIPC_ASSERT(dt_attr, "Scene config must have a 'dt' attribute.");
        auto newton_velocity_tol_attr = config.find<Float>("newton/velocity_tol");
        newton_velocity_tol = newton_velocity_tol_attr->view()[0];
        auto rel_attr = config.find<Float>("newton/velocity_tol_relative");
        velocity_tol_relative = rel_attr ? rel_attr->view()[0] : 0.0;
    }
    void do_init(InitInfo& info) override {}

    void do_pre_newton(PreNewtonInfo& info) override
    {
        res0  = 0.0;
        frame = info.frame();
    }

    void do_launch_check(LaunchCheckInfo& info) override
    {
        launched_res.resize_discard(1);
        vertex_manager->launch_axis_max_displacement(
            cuda_tool::VarView<Float>{launched_res.device_data()}, info.stream());
        launched_res.enqueue_copy(info.stream());
    }

    void do_check(CheckResultInfo& info) override
    {
        // scene-relative mode (Stiff-GIPC convention):
        // abs_tol = velocity_tol_relative * scene_diagonal * dt
        Float vel_tol = newton_velocity_tol;
        if(velocity_tol_relative > 0.0)
            vel_tol = velocity_tol_relative * vertex_manager->scene_diagonal();
        abs_tol          = vel_tol * dt_attr->view()[0];
        res              = info.launched() ? launched_res[0] :
                                             vertex_manager->compute_axis_max_displacement();
        auto newton_iter = info.newton_iter();
        if(newton_iter == 0)
            res0 = res;  // record the initial residual

        auto converged = res <= abs_tol;

        info.converged(converged);
    }

    std::string do_report() override
    {
        std::string report = fmt::format("Residual/AbsTol: {}/{}", res, abs_tol);
        return report;
    }
};

REGISTER_SIM_SYSTEM(MaxTranslationChecker);
}  // namespace uipc::backend::cuda