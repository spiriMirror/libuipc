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
#include <uipc/uipc.h>

#include <cstring>
#include <vector>

namespace uipc::backend::cuda
{
class InPlacePrelaunchTestAccess
{
  public:
    struct Snapshot
    {
        std::vector<int>       rows, cols;
        std::vector<Matrix3x3> values;
        std::vector<Float>     b;
    };

    // One Newton iteration's assembly with a DyTopo-time prelaunch, as
    // SimEngine drives it, then the converted matrix and the gradient.
    // beside_blocks > 0: DyTopo assembly blocks to come, so the in-place
    // lead may prelaunch ahead of them (PrelaunchInfo::ahead_of_contact())
    static Snapshot assemble(GlobalLinearSystem& system, SizeT beside_blocks = 0)
    {
        auto& impl = system.m_impl;
        impl.prepare_prelaunch(beside_blocks);
        impl.prelaunch_ahead();
        impl.prelaunch_assembly(true);
        impl.build_linear_system();
        Snapshot s;
        auto     A = impl.bcoo_A.cview();
        s.rows.resize(A.triplet_count());
        s.cols.resize(A.triplet_count());
        s.values.resize(A.triplet_count());
        CUDA_TOOL_CHECK(cudaMemcpy(s.rows.data(),
                                   A.row_indices().data(),
                                   s.rows.size() * sizeof(int),
                                   cudaMemcpyDeviceToHost));
        CUDA_TOOL_CHECK(cudaMemcpy(s.cols.data(),
                                   A.col_indices().data(),
                                   s.cols.size() * sizeof(int),
                                   cudaMemcpyDeviceToHost));
        CUDA_TOOL_CHECK(cudaMemcpy(s.values.data(),
                                   A.values().data(),
                                   s.values.size() * sizeof(Matrix3x3),
                                   cudaMemcpyDeviceToHost));
        impl.b.copy_to(s.b);
        return s;
    }

    static SizeT triplet_capacity(GlobalLinearSystem& system)
    {
        return system.m_impl.triplet_A.triplet_capacity();
    }
};

namespace in_place_prelaunch_test
{
    // A "grower" subsystem (first in subsystem order) whose triplet and DoF
    // counts the test changes, and a "lead" subsystem that writes its
    // prefix in place (its range then goes first), from a stream held for
    // a while first, so a missing wait in the global system shows.
    struct State
    {
        bool                enabled         = false;
        bool                in_place        = false;  // the lead asks for it
        bool                ahead           = false;  // ... ahead of contact
        bool                late            = false;  // grower's half late
        int                 round           = 0;
        SizeT               grower_blocks   = 2;
        SizeT               grower_triplets = 2;
        GlobalLinearSystem* system          = nullptr;
        // lead bookkeeping
        SizeT in_place_prelaunches = 0;
        SizeT discarded            = 0;
    };

    State& state()
    {
        static State value;
        return value;
    }

    class ScopedActivation
    {
      public:
        ScopedActivation(bool in_place, bool ahead = false, bool late = false)
        {
            state()          = State{};
            state().enabled  = true;
            state().in_place = in_place;
            state().ahead    = ahead;
            state().late     = late;
        }
        ~ScopedActivation() { state() = State{}; }
    };

    constexpr int LeadBlocks = 5;
    constexpr int LeadPrefix = 2 * LeadBlocks - 1;  // diagonal + super-diagonal
    constexpr int LeadExtra  = 2;                   // written by assemble()
    constexpr int LeadTotal  = LeadPrefix + LeadExtra;

    __device__ Matrix3x3 lead_block(int k, int round)
    {
        Matrix3x3 M = Matrix3x3::Identity() * Float(10 + k + 3 * round);
        M(0, 1) = M(1, 0) = Float(0.25) * Float(k + 1);
        return M;
    }

    __global__ void spin_kernel(long long cycles)
    {
        long long t0 = clock64();
        while(clock64() - t0 < cycles)
        {
        }
    }

    // prefix: blocks (k, k) for k < LeadBlocks, then (k, k + 1)
    __global__ void lead_prefix_kernel(cuda_tool::TripletMatrixView<Float, 3> H,
                                       cuda_tool::DenseVectorView<Float>      G,
                                       int round)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I < LeadBlocks)
            H(I).write(I, I, lead_block(I, round));
        else if(I < LeadPrefix)
        {
            int k = I - LeadBlocks;
            H(I).write(k, k + 1, lead_block(k, round) * Float(0.5));
        }
        if(I < 3 * LeadBlocks)
            G(I) = Float(I + 1) * Float(round + 1);
    }

    // the part a prelaunch leaves to assemble(): two more diagonal blocks
    __global__ void lead_extra_kernel(cuda_tool::TripletMatrixView<Float, 3> H, int round)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I < LeadExtra)
            H(LeadPrefix + I).write(I, I, lead_block(7 + I, round));
    }

    // triplets [first, first + triplets) of the grower's blocks
    __global__ void grower_kernel(cuda_tool::TripletMatrixView<Float, 3> H,
                                  cuda_tool::DenseVectorView<Float>      G,
                                  int                                    blocks,
                                  int                                    first,
                                  int triplets,
                                  int round)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I < triplets)
        {
            int k = (first + I) % blocks;
            H(I).write(k, k, lead_block(20 + first + I, round));
        }
        if(G.size() > 0 && I < 3 * blocks)
            G(I) = Float(100 + I) * Float(round + 1);
    }

    SizeT grower_late_triplets()
    {
        return state().late ? state().grower_triplets / 2 : 0;
    }

    class GrowerSubsystem final : public DiagLinearSubsystem
    {
      public:
        using DiagLinearSubsystem::DiagLinearSubsystem;

      protected:
        void do_build(BuildInfo&) override
        {
            if(!state().enabled)
                throw SimSystemException("in-place prelaunch test is inactive");
            state().system = &require<GlobalLinearSystem>();
            require<LinearFusedPCG>();
        }
        void do_init(InitInfo&) override {}
        void do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo&) override
        {
        }
        void do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info) override
        {
            info.extent(3 * state().grower_blocks);
        }
        void do_report_extent(GlobalLinearSystem::DiagExtentInfo& info) override
        {
            SizeT late = info.gradient_only() ? 0 : grower_late_triplets();
            info.extent(info.gradient_only() ? 0 : state().grower_triplets - late,
                        3 * state().grower_blocks);
            if(late > 0)
                info.late_extent();
        }
        void do_assemble(GlobalLinearSystem::DiagInfo& info) override
        {
            SizeT early = info.hessians().triplet_count();
            int   n     = (int)std::max(early, 3 * state().grower_blocks);
            grower_kernel<<<(n + 63) / 64, 64>>>(info.hessians(),
                                                 info.gradients(),
                                                 (int)state().grower_blocks,
                                                 0,
                                                 (int)early,
                                                 state().round);
        }
        void do_report_late_extent(GlobalLinearSystem::LateExtentInfo& info) override
        {
            info.extent(grower_late_triplets());
        }
        void do_assemble_late(GlobalLinearSystem::DiagInfo& info) override
        {
            SizeT late  = grower_late_triplets();
            SizeT early = state().grower_triplets - late;
            CHECK(info.hessians().triplet_count() == late);
            grower_kernel<<<((int)late + 63) / 64, 64>>>(info.hessians(),
                                                         {},
                                                         (int)state().grower_blocks,
                                                         (int)early,
                                                         (int)late,
                                                         state().round);
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
        U64 get_uid() const noexcept override { return U64{120}; }
    };

    class LeadSubsystem final : public DiagLinearSubsystem
    {
      public:
        using DiagLinearSubsystem::DiagLinearSubsystem;

      protected:
        void do_build(BuildInfo&) override
        {
            if(!state().enabled)
                throw SimSystemException("in-place prelaunch test is inactive");
        }
        void do_init(InitInfo&) override {}
        void do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo&) override
        {
        }
        void do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info) override
        {
            info.extent(3 * LeadBlocks);
            if(state().in_place)
                info.prelaunch_in_place();
        }
        void do_report_extent(GlobalLinearSystem::DiagExtentInfo& info) override
        {
            info.extent(info.gradient_only() ? 0 : LeadTotal, 3 * LeadBlocks);
        }
        void do_prepare_prelaunch(GlobalLinearSystem::PrelaunchInfo&) override
        {
            m_stream.mark();
        }
        void do_prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info) override
        {
            // ahead: the whole prelaunch in the call ahead of the contact
            // launch, nothing in the call after it
            if(m_ahead_done)
            {
                m_ahead_done = false;
                return;
            }
            m_prelaunched  = false;
            const bool now = state().ahead ? info.ahead_of_contact() :
                                             info.contact_launched();
            if(!now || !info.in_place_available())
                return;
            auto         H = info.in_place_hessians(LeadPrefix);
            auto         G = info.in_place_gradients();
            cudaStream_t s = m_stream.fork_marked();
            spin_kernel<<<1, 1, 0, s>>>(20'000'000);  // about 10 ms
            lead_prefix_kernel<<<1, 64, 0, s>>>(H, G, state().round);
            info.in_place_done(s);
            info.fully_written_triplets(LeadPrefix);
            m_prelaunched = true;
            m_ahead_done  = info.ahead_of_contact();
            ++state().in_place_prelaunches;
        }
        void do_assemble(GlobalLinearSystem::DiagInfo& info) override
        {
            const bool use = m_prelaunched && info.prelaunched();
            if(m_prelaunched && !use)
            {
                ++state().discarded;
                m_stream.join();
            }
            m_prelaunched = false;
            if(!use)
            {
                info.gradients().buffer_view().fill(0.0);
                lead_prefix_kernel<<<1, 64>>>(
                    info.hessians(), info.gradients(), state().round);
            }
            lead_extra_kernel<<<1, 32>>>(info.hessians(), state().round);
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
        U64 get_uid() const noexcept override { return U64{121}; }

      private:
        cuda_tool::SideStream m_stream;
        bool                  m_prelaunched = false;
        bool                  m_ahead_done  = false;
    };

    REGISTER_SIM_SYSTEM(GrowerSubsystem);
    REGISTER_SIM_SYSTEM(LeadSubsystem);

    using Snapshot = InPlacePrelaunchTestAccess::Snapshot;

    // rounds: 0 first assembly (no previous layout), 1 in place, 2 in place
    // with the grower's triplets grown past the capacity after the prefix
    // was written, 3 a DoF layout change (the prelaunch is discarded), 4 in
    // place again
    // late: the grower's last half of the triplets is late
    // (DiagExtentInfo::late_extent()), behind the lead's range;
    // grown_triplets: the grower's triplets from round 2 (0: four times the
    // capacity)
    std::vector<Snapshot> run(bool                in_place,
                              std::vector<SizeT>* capacities     = nullptr,
                              bool                ahead          = false,
                              bool                late           = false,
                              SizeT               grown_triplets = 0)
    {
        ScopedActivation activation{in_place, ahead, late};
        const auto       workspace = fmt::format(
            "{}in-place-{}/", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE), in_place);
        core::Engine engine{"cuda", workspace};
        core::World  world{engine};
        auto         config               = core::Scene::default_config();
        config["contact"]["enable"]       = false;
        config["contact"]["constitution"] = "ipc";
        config["sanity_check"]["enable"]  = false;
        config["linear_system"]["solver"] = "fused_pcg";
        core::Scene scene{config};
        world.init(scene);
        REQUIRE(world.is_valid());
        REQUIRE(state().system != nullptr);
        auto& system = *state().system;

        std::vector<Snapshot> out;
        for(int round = 0; round < 5; ++round)
        {
            state().round = round;
            if(round == 2)
                state().grower_triplets =
                    grown_triplets ?
                        grown_triplets :
                        4 * InPlacePrelaunchTestAccess::triplet_capacity(system);
            if(round == 3)
                state().grower_blocks = 3;
            out.push_back(InPlacePrelaunchTestAccess::assemble(system, ahead ? 48 : 0));
            if(capacities)
                capacities->push_back(InPlacePrelaunchTestAccess::triplet_capacity(system));
        }
        if(in_place)
        {
            CHECK(state().in_place_prelaunches == 4);  // rounds 1-4
            CHECK(state().discarded == 1);             // round 3
        }
        return out;
    }

    template <typename T>
    bool same_bytes(const std::vector<T>& a, const std::vector<T>& b)
    {
        return a.size() == b.size()
               && std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0;
    }
}  // namespace in_place_prelaunch_test
}  // namespace uipc::backend::cuda

TEST_CASE("An in-place prelaunch assembles the same bytes as assemble()",
          "[build_solve_focused][in_place_prelaunch]")
{
    using namespace uipc::backend::cuda::in_place_prelaunch_test;
    std::vector<uipc::SizeT> capacities, reference_capacities;
    auto                     in_place  = run(true, &capacities);
    auto                     ahead     = run(true, nullptr, true);
    auto                     reference = run(false, &reference_capacities);
    // late grower blocks (the reference's sizes), also growing the triplets
    // in the late assembly
    std::vector<uipc::SizeT> late_capacities;
    const uipc::SizeT        grown = 4 * reference_capacities[1];
    auto late_in_place = run(true, &late_capacities, false, true, grown);
    auto late          = run(false, nullptr, false, true, grown);
    REQUIRE(in_place.size() == reference.size());
    REQUIRE(ahead.size() == reference.size());
    REQUIRE(late_in_place.size() == reference.size());
    REQUIRE(late.size() == reference.size());
    REQUIRE(capacities[2] > capacities[1]);  // round 2 grew the triplets
    REQUIRE(late_capacities[2] > late_capacities[1]);
    for(std::size_t r = 0; r < in_place.size(); ++r)
    {
        CAPTURE(r);
        REQUIRE(in_place[r].rows.size() > 0);
        CHECK(same_bytes(in_place[r].rows, reference[r].rows));
        CHECK(same_bytes(in_place[r].cols, reference[r].cols));
        CHECK(same_bytes(in_place[r].values, reference[r].values));
        CHECK(same_bytes(in_place[r].b, reference[r].b));
        // prelaunched ahead of the rest of the DyTopo work (prelaunch_ahead())
        CHECK(same_bytes(ahead[r].rows, reference[r].rows));
        CHECK(same_bytes(ahead[r].cols, reference[r].cols));
        CHECK(same_bytes(ahead[r].values, reference[r].values));
        CHECK(same_bytes(ahead[r].b, reference[r].b));
        // late blocks behind the other subsystems' ranges
        CHECK(same_bytes(late_in_place[r].rows, reference[r].rows));
        CHECK(same_bytes(late_in_place[r].cols, reference[r].cols));
        CHECK(same_bytes(late_in_place[r].values, reference[r].values));
        CHECK(same_bytes(late_in_place[r].b, reference[r].b));
        CHECK(same_bytes(late[r].rows, reference[r].rows));
        CHECK(same_bytes(late[r].cols, reference[r].cols));
        CHECK(same_bytes(late[r].values, reference[r].values));
        CHECK(same_bytes(late[r].b, reference[r].b));
    }
}
