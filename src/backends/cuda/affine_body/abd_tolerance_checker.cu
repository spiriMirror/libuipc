/*
 * NVIDIA contributions: tolerance-check work launched on the line-search
 * prologue stream and the read-back of its flag through mapped memory.
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
#include <affine_body/affine_body_dynamics.h>
#include <uipc/geometry/attribute_slot.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void abd_tolerance_checker_do_check_kernel(cuda_tool::CBufferView<Vector12> dqs,
                                                          cuda_tool::VarView<IndexT> success,
                                                          Float abs_tol,
                                                          int   n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const Vector12& dq            = dqs(I);
        IndexT          success_value = *success;

        // if success is already marked as failed, skip
        if(success_value == 0)
            return;

        // the first 3 components are translation, ignore
        // the rest 9 components are rotation/scaling/shear, take
        for(IndexT i = 3; i < 12; ++i)
        {
            if(abs(dq[i]) > abs_tol)
            {
                cuda_tool::atomic_exch(success.data(), 0);
                break;  // no need to check further
            }
        }
    }
}  // namespace

class ABDToleranceChecker final : public NewtonToleranceChecker
{
  public:
    using NewtonToleranceChecker::NewtonToleranceChecker;

    SimSystemSlot<AffineBodyDynamics>       affine_body_dynamics;
    S<const geometry::AttributeSlot<Float>> dt_attr;
    Float                                   transrate_tol = 0.0;
    Float                                   abs_tol       = 0.0;
    cuda_tool::DeviceVar<IndexT>            success;
    // the flag, published by a copy kernel (no copy follows it on the
    // mapped path)
    cuda_tool::ReadbackBuffer<IndexT> h_success_copy;
    IndexT h_success = 1;  // 1 means success, 0 means failure

    // Inherited via NewtonToleranceChecker
    void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = require<AffineBodyDynamics>();
        auto& config         = world().scene().config();
        dt_attr              = config.find<Float>("dt");
        UIPC_ASSERT(dt_attr, "Scene config must have a 'dt' attribute.");
        auto transrate_tol_attr = config.find<Float>("newton/transrate_tol");
        transrate_tol           = transrate_tol_attr->view()[0];
    }

    void do_init(InitInfo& info) override {}

    void do_pre_newton(PreNewtonInfo& info) override {}

    // Check dq on `s` and copy the flag to the host. The flag is reset
    // with a kernel: a pageable host-to-device copy may wait for `s`.
    void launch(cudaStream_t s)
    {
        abs_tol  = transrate_tol * dt_attr->view()[0];
        auto dqs = affine_body_dynamics->dqs();
        using namespace cuda_tool;
        BufferLaunch(s).fill(cuda_tool::BufferView<IndexT>{success.data(), 1},
                             IndexT{1});  // reset success flag

        int n = (int)dqs.size();
        if(n > 0)
        {
            auto k = abd_tolerance_checker_do_check_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, s>>>(
                dqs, success.view(), abs_tol, n);
        }

        h_success_copy.resize_discard(1);
        BufferLaunch(s).copy(cuda_tool::BufferView<IndexT>{h_success_copy.device_data(), 1},
                             cuda_tool::CBufferView<IndexT>{success.data(), 1});
        h_success_copy.enqueue_copy(s);
    }

    void do_launch_check(LaunchCheckInfo& info) override
    {
        launch(info.stream());
    }

    void do_check(CheckResultInfo& info) override
    {
        if(!info.launched())
        {
            launch(cuda_tool::default_stream());
            CUDA_TOOL_CHECK(cudaStreamSynchronize(cuda_tool::default_stream()));
        }

        bool h_success = h_success_copy[0];
        info.converged(h_success);
    }

    std::string do_report() override
    {
        return fmt::format("Tol: {}{}", (h_success ? "< " : "> "), abs_tol);
    }
};

REGISTER_SIM_SYSTEM(ABDToleranceChecker);
}  // namespace uipc::backend::cuda