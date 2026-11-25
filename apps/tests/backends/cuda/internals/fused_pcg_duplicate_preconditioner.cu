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

#include <array>
#include <cmath>
#include <vector>

namespace uipc::backend::cuda
{
class DuplicateLocalPreconditionerTestAccess
{
  public:
    static GlobalLinearSystem::Impl& impl(GlobalLinearSystem& system)
    {
        return system.m_impl;
    }

    static std::vector<Float> solve_once(GlobalLinearSystem& system)
    {
        system.solve();
        std::vector<Float> result;
        system.m_impl.x.copy_to(result);
        return result;
    }
};

namespace duplicate_preconditioner_test
{
    struct State
    {
        bool enabled              = false;
        bool distinct_eigenvalues = false;
        // Two subsystems with one preconditioner each (unique owners): the
        // fused step runs the per-segment fallback on parallel branches.
        bool split = false;
        // The first fused hook of a capture launches onto the legacy default
        // stream, which invalidates the capture (an un-plumbed callee).
        bool                escape    = false;
        Float               rhs_scale = 1;
        GlobalLinearSystem* system    = nullptr;
        std::array<bool, 2> registered{};
        SizeT               fused_hook_calls    = 0;
        SizeT               plain_apply_calls   = 0;
        SizeT               capture_apply_calls = 0;
        SizeT               escaped_captures    = 0;

        SizeT dof_count() const { return distinct_eigenvalues ? 24 : 3; }
        // DoFs [begin, end) of subsystem `owner` (1 = TestDiagonalSubsystemB)
        SizeT dof_begin(int owner) const
        {
            return owner == 0 ? 0 : dof_count() / 2;
        }
        SizeT dof_end(int owner) const
        {
            return split && owner == 0 ? dof_count() / 2 : dof_count();
        }
        Float diagonal(SizeT i) const
        {
            return distinct_eigenvalues ? Float(i + 1) : Float{1};
        }
        Float solution(SizeT i) const { return rhs_scale * Float(i + 1); }
    };

    State& state()
    {
        static State value;
        return value;
    }

    class ScopedActivation
    {
      public:
        explicit ScopedActivation(bool distinct_eigenvalues, bool split = false, bool escape = false)
        {
            state()                      = State{};
            state().enabled              = true;
            state().distinct_eigenvalues = distinct_eigenvalues;
            state().split                = split;
            state().escape               = escape;
        }
        ~ScopedActivation() { state() = State{}; }
    };

    template <int Owner>
    class TestDiagonalSubsystemT : public DiagLinearSubsystem
    {
      public:
        using DiagLinearSubsystem::DiagLinearSubsystem;

      protected:
        void do_build(BuildInfo&) override
        {
            if(!state().enabled || (Owner == 1 && !state().split))
                throw SimSystemException("duplicate-preconditioner test is inactive");
            state().system = &require<GlobalLinearSystem>();
            require<LinearFusedPCG>();
        }

        void do_init(InitInfo&) override {}
        void do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo&) override
        {
        }
        SizeT begin() const { return state().dof_begin(Owner); }
        SizeT count() const { return state().dof_end(Owner) - begin(); }
        void do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info) override
        {
            info.extent(count());
        }
        void do_report_extent(GlobalLinearSystem::DiagExtentInfo& info) override
        {
            const SizeT n = count();
            info.extent(info.gradient_only() ? 0 : n / 3, n);
        }

        void do_assemble(GlobalLinearSystem::DiagInfo& info) override
        {
            const SizeT        n = count();
            std::vector<Float> rhs(n);
            for(SizeT i = 0; i < n; ++i)
                rhs[i] = state().diagonal(begin() + i) * state().solution(begin() + i);
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
                        state().diagonal(begin() + block * 3 + component);
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
        U64 get_uid() const noexcept override { return U64{103 + Owner}; }
    };

    class TestDiagonalSubsystem final : public TestDiagonalSubsystemT<0>
    {
      public:
        using TestDiagonalSubsystemT<0>::TestDiagonalSubsystemT;
    };

    class TestDiagonalSubsystemB final : public TestDiagonalSubsystemT<1>
    {
      public:
        using TestDiagonalSubsystemT<1>::TestDiagonalSubsystemT;
    };

    __global__ void legacy_stream_kernel() {}

    // Launches onto the legacy default stream if `stream` belongs to an
    // active capture. The launch is rejected and invalidates the capture, so
    // every later call on its streams fails, the branch join's event record
    // included.
    void escape_capture(cudaStream_t stream)
    {
        cudaStreamCaptureStatus status = cudaStreamCaptureStatusNone;
        if(cudaStreamIsCapturing(stream, &status) != cudaSuccess || status != cudaStreamCaptureStatusActive)
            return;
        ++state().escaped_captures;
        legacy_stream_kernel<<<1, 1, 0, cudaStreamLegacy>>>();
    }

    class IdentityPreconditioner : public LocalPreconditioner
    {
      public:
        using LocalPreconditioner::LocalPreconditioner;

      protected:
        void do_build(BuildInfo& info) override
        {
            if(!state().enabled)
                throw SimSystemException("duplicate-preconditioner test is inactive");
            if(state().split && slot() == 1)
                info.connect(&require<TestDiagonalSubsystemB>());
            else
                info.connect(&require<TestDiagonalSubsystem>());
            state().registered[slot()] = true;
        }
        void do_init(InitInfo&) override {}
        void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo&) override
        {
        }
        void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override
        {
            if(info.stream())
                ++state().capture_apply_calls;
            else
                ++state().plain_apply_calls;
            cuda_tool::BufferLaunch(info.stream())
                .copy(info.z().buffer_view(), info.r().buffer_view());
        }
        bool do_fused_pcg_update_apply_dot(GlobalLinearSystem::FusedPcgUpdateApplyDotInfo& info) override
        {
            ++state().fused_hook_calls;
            if(state().escape)
                escape_capture(info.stream());
            return false;
        }

      private:
        virtual int slot() const noexcept = 0;
    };

    class IdentityPreconditionerA final : public IdentityPreconditioner
    {
      public:
        using IdentityPreconditioner::IdentityPreconditioner;

      private:
        int slot() const noexcept override { return 0; }
    };

    class IdentityPreconditionerB final : public IdentityPreconditioner
    {
      public:
        using IdentityPreconditioner::IdentityPreconditioner;

      private:
        int slot() const noexcept override { return 1; }
    };

    REGISTER_SIM_SYSTEM(TestDiagonalSubsystem);
    REGISTER_SIM_SYSTEM(TestDiagonalSubsystemB);
    REGISTER_SIM_SYSTEM(IdentityPreconditionerA);
    REGISTER_SIM_SYSTEM(IdentityPreconditionerB);

    void check_fusion_declined_without_work(GlobalLinearSystem& system)
    {
        // Use the real initialized dispatcher. On the original implementation this
        // finite one-step probe returns true and updates x/r twice; fail here before
        // the next complete PCG iteration could divide by pAp == 0 on A == I.
        const SizeT                         n = state().dof_count();
        cuda_tool::DeviceDenseVector<Float> x, p, r, Ap, z;
        for(auto* vector : {&x, &p, &r, &Ap, &z})
            vector->resize(n);
        x.buffer_view().fill(Float{0});
        p.buffer_view().fill(Float{1});
        r.buffer_view().fill(Float{1});
        Ap.buffer_view().fill(Float{1});
        z.buffer_view().fill(Float{7});
        cuda_tool::DeviceVar<Float>  rz{Float{1}};
        cuda_tool::DeviceVar<Float>  pAp{Float{1}};
        cuda_tool::DeviceVar<Float>  rz_new{Float{0}};
        cuda_tool::DeviceVar<IndexT> converged{IndexT{0}};

        const bool fused = DuplicateLocalPreconditionerTestAccess::impl(system).fused_pcg_update_apply_dot(
            x.view(),
            p.cview(),
            r.view(),
            Ap.cview(),
            z.view(),
            rz.view(),
            pAp.view(),
            rz_new.view(),
            converged.view(),
            nullptr);
        std::vector<Float> actual_x, actual_r, actual_z;
        x.copy_to(actual_x);
        r.copy_to(actual_r);
        z.copy_to(actual_z);
        CAPTURE(fused, state().fused_hook_calls, actual_x[0], actual_r[0]);
        REQUIRE_FALSE(fused);
        REQUIRE(state().fused_hook_calls == 0);
        REQUIRE(state().plain_apply_calls == 0);
        REQUIRE(state().capture_apply_calls == 0);
        for(SizeT i = 0; i < n; ++i)
        {
            REQUIRE(actual_x[i] == Float{0});
            REQUIRE(actual_r[i] == Float{1});
            REQUIRE(actual_z[i] == Float{7});
        }
        REQUIRE(static_cast<Float>(rz_new) == Float{0});
    }

    using Solutions = std::array<std::vector<Float>, 2>;

    Solutions solve_twice(IndexT graph_mode,
                          bool   distinct_eigenvalues,
                          bool   split  = false,
                          bool   escape = false)
    {
        ScopedActivation activation{distinct_eigenvalues, split, escape};
        const auto       workspace =
            fmt::format("{}mode-{}-distinct-{}-split-{}-escape-{}/",
                        AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE),
                        graph_mode,
                        distinct_eigenvalues,
                        split,
                        escape);
        core::Engine engine{"cuda", workspace};
        core::World  world{engine};
        auto         config               = core::Scene::default_config();
        config["contact"]["enable"]       = false;
        config["contact"]["constitution"] = "ipc";
        config["sanity_check"]["enable"]  = false;
        config["linear_system"]["solver"] = "fused_pcg";
        config["linear_system"]["use_cuda_graph"] = graph_mode;
        constexpr IndexT Interval                 = 3;
        config["linear_system"]["check_interval"] = Interval;
        config["linear_system"]["tol_rate"]       = Float{1e-20};
        core::Scene scene{config};
        world.init(scene);
        REQUIRE(world.is_valid());
        REQUIRE(state().system != nullptr);
        REQUIRE(state().registered[0]);
        REQUIRE(state().registered[1]);
        auto& system = *state().system;
        if(graph_mode == 1 && !split)
            check_fusion_declined_without_work(system);

        Solutions result;
        SizeT     plain_applies = 0;
        for(SizeT solve = 0; solve < result.size(); ++solve)
        {
            state().rhs_scale = solve == 0 ? Float{1} : Float{2};
            result[solve] = DuplicateLocalPreconditionerTestAccess::solve_once(system);
            REQUIRE(result[solve].size() == state().dof_count());
            Float residual_squared = 0;
            Float rhs_squared      = 0;
            for(SizeT i = 0; i < result[solve].size(); ++i)
            {
                CAPTURE(graph_mode, distinct_eigenvalues, solve, i);
                REQUIRE(std::isfinite(result[solve][i]));
                REQUIRE(result[solve][i] == Catch::Approx(state().solution(i)).epsilon(1e-8));
                const Float rhs = state().diagonal(i) * state().solution(i);
                const Float residual = rhs - state().diagonal(i) * result[solve][i];
                residual_squared += residual * residual;
                rhs_squared += rhs * rhs;
            }
            REQUIRE(std::sqrt(residual_squared / rhs_squared) < Float{2e-9});
            // With unique owners the fused step asks each preconditioner once
            // per captured slot (it declines and runs the segment fallback).
            REQUIRE(state().fused_hook_calls
                    == (split ? state().capture_apply_calls - (graph_mode == 2 ? 2 : 0) : 0));

            if(distinct_eigenvalues)
                REQUIRE(system.last_solve_iterations() > static_cast<SizeT>(Interval));
            if(escape)
            {
                // Each capture fails at the first fused step of its body (in
                // mode 2 the WHILE loop body, then the replay block) and stays
                // disabled: every iteration of both solves runs on the plain
                // path. The second solve repeats the assembly (CUB sorts and
                // scans in the library's runtime) and must still be exact.
                const SizeT failed = graph_mode == 2 ? 2 : 1;
                REQUIRE(state().escaped_captures == failed);
                REQUIRE(state().capture_apply_calls
                        == 2 * failed + (graph_mode == 2 ? 2 : 0));
                plain_applies += 2 * (1 + system.last_solve_iterations());
                REQUIRE(state().plain_apply_calls == plain_applies);
                // The escaped launch and the identity apply's copy in the dead
                // capture ran in this executable's own CUDA runtime (static
                // cudart), which the library's recovery does not clear.
                cudaGetLastError();
            }
            else if(graph_mode == 1)
            {
                // Only initialization applies execute on the host. Exactly one
                // three-slot capture, reused across chunks and the second solve,
                // rules out a silent plain-launch fallback masking the regression.
                REQUIRE(state().plain_apply_calls == 2 * (solve + 1));
                REQUIRE(state().capture_apply_calls == 2 * static_cast<SizeT>(Interval));
            }
#if CUDA_TOOL_GRAPH_WHILE
            else if(graph_mode == 2)
            {
                // A conditional graph captures each preconditioner once in the
                // setup chain and once per ping-pong slot of the two-iteration
                // loop body. Reusing that graph for the second solve must not
                // invoke any capture callback again.
                REQUIRE(state().plain_apply_calls == 0);
                REQUIRE(state().capture_apply_calls == 6);
            }
#endif
            else
            {
                REQUIRE(state().capture_apply_calls == 0);
            }
        }
        return result;
    }
}  // namespace duplicate_preconditioner_test
}  // namespace uipc::backend::cuda

TEST_CASE("Fused PCG preserves sequential local preconditioners on one subsystem",
          "[build_solve_focused][fused_pcg][duplicate_preconditioner]")
{
    using namespace uipc;
    using namespace uipc::backend::cuda::duplicate_preconditioner_test;

    bool distinct_eigenvalues = false;
    SECTION("identity system with converged trailing slots") {}
    SECTION("multiple graph chunks and a repeated solve")
    {
        distinct_eigenvalues = true;
    }

    const auto plain             = solve_twice(0, distinct_eigenvalues);
    const auto graph             = solve_twice(1, distinct_eigenvalues);
    auto       compare_solutions = [&](const Solutions& actual)
    {
        for(SizeT solve = 0; solve < plain.size(); ++solve)
        {
            REQUIRE(actual[solve].size() == plain[solve].size());
            for(SizeT i = 0; i < plain[solve].size(); ++i)
                REQUIRE(actual[solve][i] == Catch::Approx(plain[solve][i]).epsilon(1e-8));
        }
    };
    compare_solutions(graph);

#if CUDA_TOOL_GRAPH_WHILE
    if(uipc::backend::cuda_tool::GraphWhile::runtime_supported())
        compare_solutions(solve_twice(2, distinct_eigenvalues));
#endif
}

TEST_CASE("Fused PCG runs the segments of unique local preconditioners as parallel branches",
          "[build_solve_focused][fused_pcg][duplicate_preconditioner]")
{
    using namespace uipc;
    using namespace uipc::backend::cuda::duplicate_preconditioner_test;

    // Two subsystems with distinct eigenvalues and one identity preconditioner
    // each: several PCG iterations whose fused step forks one segment onto a
    // side stream. The captured graphs must reproduce the plain solve.
    const auto plain             = solve_twice(0, true, true);
    auto       compare_solutions = [&](const Solutions& actual)
    {
        for(SizeT solve = 0; solve < plain.size(); ++solve)
        {
            REQUIRE(actual[solve].size() == plain[solve].size());
            for(SizeT i = 0; i < plain[solve].size(); ++i)
                REQUIRE(actual[solve][i] == Catch::Approx(plain[solve][i]).epsilon(1e-8));
        }
    };
    compare_solutions(solve_twice(1, true, true));

#if CUDA_TOOL_GRAPH_WHILE
    if(uipc::backend::cuda_tool::GraphWhile::runtime_supported())
        compare_solutions(solve_twice(2, true, true));
#endif
}

TEST_CASE("Fused PCG falls back to plain launches when a parallel branch invalidates the capture",
          "[build_solve_focused][fused_pcg][duplicate_preconditioner]")
{
    using namespace uipc;
    using namespace uipc::backend::cuda::duplicate_preconditioner_test;

    // The forked segment's fused hook launches onto the legacy default stream
    // inside the capture, so the branch join's checked event record throws on
    // the invalidated capture. The capture must end and take its fallback to
    // plain launches, which reproduce the plain solve.
    const auto plain             = solve_twice(0, true, true);
    auto       compare_solutions = [&](const Solutions& actual)
    {
        for(SizeT solve = 0; solve < plain.size(); ++solve)
        {
            REQUIRE(actual[solve].size() == plain[solve].size());
            for(SizeT i = 0; i < plain[solve].size(); ++i)
                REQUIRE(actual[solve][i] == Catch::Approx(plain[solve][i]).epsilon(1e-8));
        }
    };
    compare_solutions(solve_twice(1, true, true, true));

#if CUDA_TOOL_GRAPH_WHILE
    if(uipc::backend::cuda_tool::GraphWhile::runtime_supported())
        compare_solutions(solve_twice(2, true, true, true));
#endif
}
