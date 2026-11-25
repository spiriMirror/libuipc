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
#include <linear_system/diag_linear_subsystem.h>
#include <linear_system/linear_fused_pcg.h>
#include <linear_system/local_preconditioner.h>
#include <uipc/uipc.h>

#include <cmath>
#include <iterator>
#include <string_view>
#include <vector>

namespace uipc::backend::cuda
{
class PreconditionerReuseTestAccess
{
  public:
    // One Newton-iteration solve, keeping the previous preconditioner when
    // `keep` holds (GlobalLinearSystem::solve() passes
    // keeps_preconditioner(newton_iter) there).
    static std::vector<Float> solve(GlobalLinearSystem& system, bool keep)
    {
        auto& impl                 = system.m_impl;
        impl.last_solve_iterations = 0;
        impl.build_linear_system(keep);
        impl.solve_linear_system();
        std::vector<Float> result;
        impl.x.copy_to(result);
        return result;
    }

    static SizeT iterations(GlobalLinearSystem& system)
    {
        // reads a pending (while-loop) count back
        return system.last_solve_iterations();
    }

    // Whether solve() keeps preconditioners on keeps_preconditioner()
    // iterations (linear_system/keep_preconditioner, read at build).
    static bool reuse_enabled(GlobalLinearSystem& system)
    {
        return system.m_impl.preconditioner_reuse;
    }
};

namespace preconditioner_reuse_test
{
    // A diagonal system diag(d) x = d * s with a Jacobi preconditioner that
    // inverts the diagonal current at its assembly. A fresh preconditioner
    // solves the system in one PCG iteration; a kept one is stale after the
    // diagonal is distorted (M^-1 A then has eigenvalues spread over 1..10)
    // and needs many more.
    struct State
    {
        bool                enabled    = false;
        SizeT               dof_count  = 24;
        bool                distorted  = false;
        GlobalLinearSystem* system     = nullptr;
        SizeT               assemblies = 0;

        Float diagonal(SizeT i) const
        {
            return 1 + Float(i + 1) * (distorted ? Float(1 + 3 * (i % 4)) : Float{1});
        }
        Float solution(SizeT i) const { return Float(i % 5 + 1); }
    };

    State& state()
    {
        static State value;
        return value;
    }

    class ScopedActivation
    {
      public:
        ScopedActivation()
        {
            state()         = State{};
            state().enabled = true;
        }
        ~ScopedActivation() { state() = State{}; }
    };

    __global__ void jacobi_apply_kernel(cuda_tool::CDenseVectorView<Float> r,
                                        cuda_tool::DenseVectorView<Float>  z,
                                        cuda_tool::CBufferView<Float> inverse,
                                        int                           n)
    {
        const int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i < n)
            z(i) = inverse(i) * r(i);
    }

    class ReuseTestSubsystem final : public DiagLinearSubsystem
    {
      public:
        using DiagLinearSubsystem::DiagLinearSubsystem;

      protected:
        void do_build(BuildInfo&) override
        {
            if(!state().enabled)
                throw SimSystemException("preconditioner-reuse test is inactive");
            state().system = &require<GlobalLinearSystem>();
            require<LinearFusedPCG>();
        }

        void do_init(InitInfo&) override {}
        void do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo&) override
        {
        }
        void do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info) override
        {
            info.extent(state().dof_count);
        }
        void do_report_extent(GlobalLinearSystem::DiagExtentInfo& info) override
        {
            const SizeT n = state().dof_count;
            info.extent(info.gradient_only() ? 0 : n / 3, n);
        }

        void do_assemble(GlobalLinearSystem::DiagInfo& info) override
        {
            const SizeT        n = state().dof_count;
            std::vector<Float> rhs(n);
            for(SizeT i = 0; i < n; ++i)
                rhs[i] = state().diagonal(i) * state().solution(i);
            info.gradients().buffer_view().copy_from(rhs.data());
            if(info.gradient_only())
                return;

            auto                   hessians = info.hessians();
            std::vector<int>       rows(n / 3);
            std::vector<Matrix3x3> values(n / 3, Matrix3x3::Zero());
            for(SizeT block = 0; block < n / 3; ++block)
            {
                rows[block] = hessians.submatrix_offset().x + static_cast<int>(block);
                for(int component = 0; component < 3; ++component)
                    values[block](component, component) =
                        state().diagonal(block * 3 + component);
            }
            hessians.row_indices().copy_from(rows.data());
            hessians.col_indices().copy_from(rows.data());
            hessians.values().copy_from(values.data());
        }

        void do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info) override
        {
            info.satisfied(true);
        }
        void do_retrieve_solution(GlobalLinearSystem::SolutionInfo&) override {}
        Float do_diag_norm(GlobalLinearSystem::DiagNormInfo&) override
        {
            return Float{1};
        }
        Float do_mass_norm(GlobalLinearSystem::DiagNormInfo&) override
        {
            return Float{1};
        }
        U64 get_uid() const noexcept override { return U64{104}; }
    };

    class CountingJacobiPreconditioner final : public LocalPreconditioner
    {
      public:
        using LocalPreconditioner::LocalPreconditioner;

      protected:
        void do_build(BuildInfo& info) override
        {
            if(!state().enabled)
                throw SimSystemException("preconditioner-reuse test is inactive");
            info.connect(&require<ReuseTestSubsystem>());
        }
        void do_init(InitInfo&) override {}
        void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo&) override
        {
            ++state().assemblies;
            const SizeT        n = state().dof_count;
            std::vector<Float> inverse(n);
            for(SizeT i = 0; i < n; ++i)
                inverse[i] = Float{1} / state().diagonal(i);
            m_inverse.resize(n);
            m_inverse.view().copy_from(inverse.data());
        }
        void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override
        {
            const int n = (int)info.z().size();
            if(n == 0)
                return;
            jacobi_apply_kernel<<<(n + 127) / 128, 128, 0, info.stream()>>>(
                info.r(), info.z(), m_inverse.cview(), n);
        }

      private:
        cuda_tool::DeviceBuffer<Float> m_inverse;
    };

    REGISTER_SIM_SYSTEM(ReuseTestSubsystem);
    REGISTER_SIM_SYSTEM(CountingJacobiPreconditioner);

    void require_solution(const std::vector<Float>& x)
    {
        REQUIRE(x.size() == state().dof_count);
        for(SizeT i = 0; i < x.size(); ++i)
        {
            CAPTURE(i);
            REQUIRE(std::isfinite(x[i]));
            REQUIRE(x[i] == Catch::Approx(state().solution(i)).epsilon(1e-8));
        }
    }

    Json test_config(IndexT graph_mode)
    {
        auto config                       = core::Scene::default_config();
        config["contact"]["enable"]       = false;
        config["contact"]["constitution"] = "ipc";
        config["sanity_check"]["enable"]  = false;
        config["linear_system"]["solver"] = "fused_pcg";
        config["linear_system"]["use_cuda_graph"] = graph_mode;
        config["linear_system"]["tol_rate"]       = Float{1e-20};
        return config;
    }

    // Reuse is off unless linear_system/keep_preconditioner is 1.
    bool reuse_enabled(const Json& config, std::string_view name)
    {
        ScopedActivation activation;
        const auto       workspace =
            fmt::format("{}{}/", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE), name);
        core::Engine engine{"cuda", workspace};
        core::World  world{engine};
        core::Scene  scene{config};
        world.init(scene);
        REQUIRE(world.is_valid());
        REQUIRE(state().system != nullptr);
        return PreconditionerReuseTestAccess::reuse_enabled(*state().system);
    }

    void run(IndexT graph_mode)
    {
        ScopedActivation activation;
        const auto       workspace =
            fmt::format("{}mode-{}/", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE), graph_mode);
        core::Engine engine{"cuda", workspace};
        core::World  world{engine};
        core::Scene  scene{test_config(graph_mode)};
        world.init(scene);
        REQUIRE(world.is_valid());
        REQUIRE(state().system != nullptr);
        auto& system = *state().system;

        // A fresh Jacobi preconditioner is exact on a diagonal system (the
        // reported count is the first host check: 1 iteration in mode 2, one
        // check interval otherwise).
        state().assemblies = 0;
        require_solution(PreconditionerReuseTestAccess::solve(system, false));
        REQUIRE(state().assemblies == 1);
        const SizeT fresh_iterations = PreconditionerReuseTestAccess::iterations(system);
        REQUIRE(fresh_iterations <= 5);

        // The next Newton iteration changes the matrix. A kept preconditioner
        // is not reassembled; PCG still reaches the same tolerance, through
        // more iterations of the stale preconditioner.
        state().distorted = true;
        require_solution(PreconditionerReuseTestAccess::solve(system, true));
        REQUIRE(state().assemblies == 1);
        REQUIRE(PreconditionerReuseTestAccess::iterations(system) > fresh_iterations);

        // Without keeping, the preconditioner follows the matrix again.
        require_solution(PreconditionerReuseTestAccess::solve(system, false));
        REQUIRE(state().assemblies == 2);
        REQUIRE(PreconditionerReuseTestAccess::iterations(system) == fresh_iterations);

        // A DoF layout change always assembles, even when keeping is asked.
        state().dof_count = 30;
        require_solution(PreconditionerReuseTestAccess::solve(system, true));
        REQUIRE(state().assemblies == 3);
        REQUIRE(PreconditionerReuseTestAccess::iterations(system) == fresh_iterations);
    }
}  // namespace preconditioner_reuse_test
}  // namespace uipc::backend::cuda

TEST_CASE("Preconditioner reuse keeps every other Newton iteration's preconditioner",
          "[build_solve_focused][fused_pcg][preconditioner_reuse]")
{
    using uipc::backend::cuda::GlobalLinearSystem;
    constexpr bool expected[] = {false, false, false, true, false, true, false, true, false};
    for(uipc::SizeT k = 0; k < std::size(expected); ++k)
    {
        CAPTURE(k);
        REQUIRE(GlobalLinearSystem::keeps_preconditioner(k) == expected[k]);
    }
}

TEST_CASE("Preconditioner reuse is opt-in through linear_system/keep_preconditioner",
          "[build_solve_focused][fused_pcg][preconditioner_reuse]")
{
    using namespace uipc::backend::cuda::preconditioner_reuse_test;
    auto config = test_config(1);
    REQUIRE(config["linear_system"]["keep_preconditioner"] == 0);
    REQUIRE_FALSE(reuse_enabled(config, "keep-default"));

    config["linear_system"]["keep_preconditioner"] = 1;
    REQUIRE(reuse_enabled(config, "keep-on"));
}

TEST_CASE("A kept preconditioner preserves the PCG tolerance",
          "[build_solve_focused][fused_pcg][preconditioner_reuse]")
{
    using namespace uipc::backend::cuda::preconditioner_reuse_test;
    run(0);
    run(1);
#if CUDA_TOOL_GRAPH_WHILE
    if(uipc::backend::cuda_tool::GraphWhile::runtime_supported())
        run(2);
#endif
}
