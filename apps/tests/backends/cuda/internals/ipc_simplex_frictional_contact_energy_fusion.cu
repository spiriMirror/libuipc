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
#include <contact_system/contact_models/codim_ipc_simplex_frictional_contact_function.h>
#include <contact_system/contact_models/ipc_simplex_frictional_contact_energy.h>
#include <cuda_tool/cuda_tool.h>
#include <type_define.h>
#include <utils/codim_thickness.h>
#include <utils/primitive_d_hat.h>

#include <array>
#include <cmath>
#include <cstring>
#include <vector>

using namespace uipc;
using namespace uipc::backend::cuda;
using namespace uipc::backend::cuda_tool;

namespace
{
struct Counts
{
    int pt = 0;
    int ee = 0;
    int pe = 0;
    int pp = 0;
};

__global__ void friction_energy_oracle_pt_kernel(IPCSimplexFrictionalContactEnergyLaunchInfo info,
                                                 int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;

    using namespace sym::codim_ipc_contact;
    auto        table     = info.contact_tabular.viewer();
    const auto& PT        = info.PTs(i);
    Vector4i    cids      = {info.contact_element_ids(PT[0]),
                             info.contact_element_ids(PT[1]),
                             info.contact_element_ids(PT[2]),
                             info.contact_element_ids(PT[3])};
    auto        coeff     = PT_contact_coeff(table, cids);
    Float       kt2       = coeff.kappa * info.dt * info.dt;
    Float       mu        = coeff.mu;
    const auto& prev_P    = info.prev_positions(PT[0]);
    const auto& prev_T0   = info.prev_positions(PT[1]);
    const auto& prev_T1   = info.prev_positions(PT[2]);
    const auto& prev_T2   = info.prev_positions(PT[3]);
    const auto& P         = info.positions(PT[0]);
    const auto& T0        = info.positions(PT[1]);
    const auto& T1        = info.positions(PT[2]);
    const auto& T2        = info.positions(PT[3]);
    Float       thickness = PT_thickness(info.thicknesses(PT[0]),
                                   info.thicknesses(PT[1]),
                                   info.thicknesses(PT[2]),
                                   info.thicknesses(PT[3]));
    Float       d_hat     = PT_d_hat(
        info.d_hats(PT[0]), info.d_hats(PT[1]), info.d_hats(PT[2]), info.d_hats(PT[3]));
    info.PT_energies(i) = PT_friction_energy(
        kt2, d_hat, thickness, mu, info.eps_velocity * info.dt, prev_P, prev_T0, prev_T1, prev_T2, P, T0, T1, T2);
}

__global__ void friction_energy_oracle_ee_kernel(IPCSimplexFrictionalContactEnergyLaunchInfo info,
                                                 int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;

    using namespace sym::codim_ipc_contact;
    auto           table     = info.contact_tabular.viewer();
    const auto&    EE        = info.EEs(i);
    Vector4i       cids      = {info.contact_element_ids(EE[0]),
                                info.contact_element_ids(EE[1]),
                                info.contact_element_ids(EE[2]),
                                info.contact_element_ids(EE[3])};
    auto           coeff     = EE_contact_coeff(table, cids);
    Float          kt2       = coeff.kappa * info.dt * info.dt;
    Float          mu        = coeff.mu;
    const Vector3& rest_Ea0  = info.rest_positions(EE[0]);
    const Vector3& rest_Ea1  = info.rest_positions(EE[1]);
    const Vector3& rest_Eb0  = info.rest_positions(EE[2]);
    const Vector3& rest_Eb1  = info.rest_positions(EE[3]);
    const Vector3& prev_Ea0  = info.prev_positions(EE[0]);
    const Vector3& prev_Ea1  = info.prev_positions(EE[1]);
    const Vector3& prev_Eb0  = info.prev_positions(EE[2]);
    const Vector3& prev_Eb1  = info.prev_positions(EE[3]);
    const Vector3& Ea0       = info.positions(EE[0]);
    const Vector3& Ea1       = info.positions(EE[1]);
    const Vector3& Eb0       = info.positions(EE[2]);
    const Vector3& Eb1       = info.positions(EE[3]);
    Float          thickness = EE_thickness(info.thicknesses(EE[0]),
                                   info.thicknesses(EE[1]),
                                   info.thicknesses(EE[2]),
                                   info.thicknesses(EE[3]));
    Float          d_hat     = EE_d_hat(
        info.d_hats(EE[0]), info.d_hats(EE[1]), info.d_hats(EE[2]), info.d_hats(EE[3]));
    Float eps_x;
    distance::edge_edge_mollifier_threshold(
        rest_Ea0, rest_Ea1, rest_Eb0, rest_Eb1, static_cast<Float>(1e-3), eps_x);
    if(distance::need_mollify(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, eps_x))
        info.EE_energies(i) = 0;
    else
        info.EE_energies(i) = EE_friction_energy(kt2,
                                                 d_hat,
                                                 thickness,
                                                 mu,
                                                 info.eps_velocity * info.dt,
                                                 prev_Ea0,
                                                 prev_Ea1,
                                                 prev_Eb0,
                                                 prev_Eb1,
                                                 Ea0,
                                                 Ea1,
                                                 Eb0,
                                                 Eb1);
}

__global__ void friction_energy_oracle_pe_kernel(IPCSimplexFrictionalContactEnergyLaunchInfo info,
                                                 int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;

    using namespace sym::codim_ipc_contact;
    auto           table     = info.contact_tabular.viewer();
    const auto&    PE        = info.PEs(i);
    Vector3i       cids      = {info.contact_element_ids(PE[0]),
                                info.contact_element_ids(PE[1]),
                                info.contact_element_ids(PE[2])};
    auto           coeff     = PE_contact_coeff(table, cids);
    Float          kt2       = coeff.kappa * info.dt * info.dt;
    Float          mu        = coeff.mu;
    const Vector3& prev_P    = info.prev_positions(PE[0]);
    const Vector3& prev_E0   = info.prev_positions(PE[1]);
    const Vector3& prev_E1   = info.prev_positions(PE[2]);
    const Vector3& P         = info.positions(PE[0]);
    const Vector3& E0        = info.positions(PE[1]);
    const Vector3& E1        = info.positions(PE[2]);
    Float          thickness = PE_thickness(info.thicknesses(PE[0]),
                                   info.thicknesses(PE[1]),
                                   info.thicknesses(PE[2]));
    Float d_hat = PE_d_hat(info.d_hats(PE[0]), info.d_hats(PE[1]), info.d_hats(PE[2]));
    info.PE_energies(i) = PE_friction_energy(
        kt2, d_hat, thickness, mu, info.eps_velocity * info.dt, prev_P, prev_E0, prev_E1, P, E0, E1);
}

__global__ void friction_energy_oracle_pp_kernel(IPCSimplexFrictionalContactEnergyLaunchInfo info,
                                                 int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;

    using namespace sym::codim_ipc_contact;
    auto        table = info.contact_tabular.viewer();
    const auto& PP    = info.PPs(i);
    Vector2i cids = {info.contact_element_ids(PP[0]), info.contact_element_ids(PP[1])};
    auto           coeff   = PP_contact_coeff(table, cids);
    Float          kt2     = coeff.kappa * info.dt * info.dt;
    Float          mu      = coeff.mu;
    const Vector3& prev_P0 = info.prev_positions(PP[0]);
    const Vector3& prev_P1 = info.prev_positions(PP[1]);
    const Vector3& P0      = info.positions(PP[0]);
    const Vector3& P1      = info.positions(PP[1]);
    Float thickness = PP_thickness(info.thicknesses(PP[0]), info.thicknesses(PP[1]));
    Float d_hat         = PP_d_hat(info.d_hats(PP[0]), info.d_hats(PP[1]));
    info.PP_energies(i) = PP_friction_energy(
        kt2, d_hat, thickness, mu, info.eps_velocity * info.dt, prev_P0, prev_P1, P0, P1);
}

void launch_four_kernel_oracle(const IPCSimplexFrictionalContactEnergyLaunchInfo& info)
{
    int n = static_cast<int>(info.PTs.size());
    if(n > 0)
        friction_energy_oracle_pt_kernel<<<best_grid_dim(n, friction_energy_oracle_pt_kernel), best_block_dim(friction_energy_oracle_pt_kernel), 0, nullptr>>>(
            info, n);
    n = static_cast<int>(info.EEs.size());
    if(n > 0)
        friction_energy_oracle_ee_kernel<<<best_grid_dim(n, friction_energy_oracle_ee_kernel), best_block_dim(friction_energy_oracle_ee_kernel), 0, nullptr>>>(
            info, n);
    n = static_cast<int>(info.PEs.size());
    if(n > 0)
        friction_energy_oracle_pe_kernel<<<best_grid_dim(n, friction_energy_oracle_pe_kernel), best_block_dim(friction_energy_oracle_pe_kernel), 0, nullptr>>>(
            info, n);
    n = static_cast<int>(info.PPs.size());
    if(n > 0)
        friction_energy_oracle_pp_kernel<<<best_grid_dim(n, friction_energy_oracle_pp_kernel), best_block_dim(friction_energy_oracle_pp_kernel), 0, nullptr>>>(
            info, n);
}

template <typename T>
void resize_and_copy_prefix(DeviceBuffer<T>& buffer, const std::vector<T>& values, int count)
{
    buffer.resize(count);
    if(count > 0)
    {
        REQUIRE(!values.empty());
        // Repeat the original valid geometry for larger CTA boundary cases.
        // Every contact still owns a distinct output energy slot.
        std::vector<T> repeated(static_cast<std::size_t>(count));
        for(std::size_t i = 0; i < repeated.size(); ++i)
            repeated[i] = values[i % values.size()];
        buffer.view().copy_from(repeated.data());
    }
}

class EnergyFixture
{
  public:
    static constexpr int MaxPerType   = 5;
    static constexpr int MaxTestCount = 513;

    EnergyFixture()
        : contact_tabular(Extent2D{1, 1})
    {
        ContactCoeff coeff;
        coeff.kappa = 3.25;
        coeff.mu    = 0.65;
        contact_tabular.view().copy_from(&coeff);

        std::vector<Vector3> positions;
        positions.reserve(65);
        for(int i = 0; i < MaxPerType; ++i)
        {
            const Float x = Float{3.0} * i;
            const Float d = Float{0.20} + Float{0.04} * i;
            const int   b = static_cast<int>(positions.size());
            positions.push_back(Vector3{x + Float{0.2}, Float{0.2}, d});
            positions.push_back(Vector3{x, Float{0.0}, Float{0.0}});
            positions.push_back(Vector3{x + Float{1.0}, Float{0.0}, Float{0.0}});
            positions.push_back(Vector3{x, Float{1.0}, Float{0.0}});
            PTs.push_back(Vector4i{b, b + 1, b + 2, b + 3});
        }
        for(int i = 0; i < MaxPerType; ++i)
        {
            const Float x = Float{20.0} + Float{3.0} * i;
            const Float d = Float{0.22} + Float{0.04} * i;
            const int   b = static_cast<int>(positions.size());
            positions.push_back(Vector3{x - Float{1.0}, Float{0.0}, Float{0.0}});
            positions.push_back(Vector3{x + Float{1.0}, Float{0.0}, Float{0.0}});
            positions.push_back(Vector3{x, Float{-1.0}, d});
            positions.push_back(Vector3{x, Float{1.0}, d});
            EEs.push_back(Vector4i{b, b + 1, b + 2, b + 3});
        }
        for(int i = 0; i < MaxPerType; ++i)
        {
            const Float x = Float{40.0} + Float{3.0} * i;
            const Float d = Float{0.24} + Float{0.04} * i;
            const int   b = static_cast<int>(positions.size());
            positions.push_back(Vector3{x, d, Float{0.0}});
            positions.push_back(Vector3{x - Float{1.0}, Float{0.0}, Float{0.0}});
            positions.push_back(Vector3{x + Float{1.0}, Float{0.0}, Float{0.0}});
            PEs.push_back(Vector3i{b, b + 1, b + 2});
        }
        for(int i = 0; i < MaxPerType; ++i)
        {
            const Float x = Float{60.0} + Float{3.0} * i;
            const Float d = Float{0.26} + Float{0.04} * i;
            const int   b = static_cast<int>(positions.size());
            positions.push_back(Vector3{x, Float{0.0}, Float{0.0}});
            positions.push_back(Vector3{x + d, Float{0.0}, Float{0.0}});
            PPs.push_back(Vector2i{b, b + 1});
        }

        auto prev_positions = positions;
        for(const auto& PT : PTs)
            positions[PT[0]](0) += Float{0.015};
        for(const auto& EE : EEs)
            positions[EE[2]](0) += Float{0.017};
        for(const auto& PE : PEs)
            positions[PE[0]](0) += Float{0.019};
        for(const auto& PP : PPs)
            positions[PP[0]](1) += Float{0.021};

        std::vector<IndexT> contact_ids(positions.size(), 0);
        std::vector<Float>  thicknesses(positions.size(), 0.0);
        std::vector<Float>  d_hats(positions.size(), 1.0);
        d_positions.copy_from(positions.data(), positions.size());
        d_prev_positions.copy_from(prev_positions.data(), prev_positions.size());
        d_rest_positions.copy_from(prev_positions.data(), prev_positions.size());
        d_contact_ids.copy_from(contact_ids.data(), contact_ids.size());
        d_thicknesses.copy_from(thicknesses.data(), thicknesses.size());
        d_d_hats.copy_from(d_hats.data(), d_hats.size());
    }

    void compare(Counts counts)
    {
        REQUIRE(counts.pt >= 0);
        REQUIRE(counts.ee >= 0);
        REQUIRE(counts.pe >= 0);
        REQUIRE(counts.pp >= 0);
        REQUIRE(counts.pt <= MaxTestCount);
        REQUIRE(counts.ee <= MaxTestCount);
        REQUIRE(counts.pe <= MaxTestCount);
        REQUIRE(counts.pp <= MaxTestCount);
        resize_and_copy_prefix(d_PTs, PTs, counts.pt);
        resize_and_copy_prefix(d_EEs, EEs, counts.ee);
        resize_and_copy_prefix(d_PEs, PEs, counts.pe);
        resize_and_copy_prefix(d_PPs, PPs, counts.pp);
        resize_outputs(reference_PT, fused_PT, counts.pt);
        resize_outputs(reference_EE, fused_EE, counts.ee);
        resize_outputs(reference_PE, fused_PE, counts.pe);
        resize_outputs(reference_PP, fused_PP, counts.pp);
        d_lagged.resize(counts.pt + counts.ee + counts.pe + counts.pp);

        auto reference_info =
            make_info(reference_PT, reference_EE, reference_PE, reference_PP);
        auto fused_info = make_info(fused_PT, fused_EE, fused_PE, fused_PP);
        launch_four_kernel_oracle(reference_info);
        // The production launch computes the lagged friction quantities and
        // evaluates from them; the oracle recomputes everything per pair.
        launch_ipc_simplex_frictional_contact_energy(fused_info);
        compare_buffer(reference_PT, fused_PT);
        compare_buffer(reference_EE, fused_EE);
        compare_buffer(reference_PE, fused_PE);
        compare_buffer(reference_PP, fused_PP);

        // A later evaluation of the frame reuses them.
        resize_outputs(reference_PT, fused_PT, counts.pt);
        resize_outputs(reference_EE, fused_EE, counts.ee);
        resize_outputs(reference_PE, fused_PE, counts.pe);
        resize_outputs(reference_PP, fused_PP, counts.pp);
        launch_four_kernel_oracle(reference_info);
        fused_info.update_lagged = false;
        launch_ipc_simplex_frictional_contact_energy(fused_info);
        compare_buffer(reference_PT, fused_PT);
        compare_buffer(reference_EE, fused_EE);
        compare_buffer(reference_PE, fused_PE);
        compare_buffer(reference_PP, fused_PP);
    }

  private:
    static void resize_outputs(DeviceBuffer<Float>& reference, DeviceBuffer<Float>& fused, int count)
    {
        reference.resize(count);
        fused.resize(count);
        if(count > 0)
        {
            reference.fill(Float{-111.0});
            fused.fill(Float{-222.0});
        }
    }

    static void compare_buffer(DeviceBuffer<Float>& reference, DeviceBuffer<Float>& fused)
    {
        std::vector<Float> reference_host;
        std::vector<Float> fused_host;
        reference.copy_to(reference_host);
        fused.copy_to(fused_host);
        REQUIRE(reference_host.size() == fused_host.size());
        if(!reference_host.empty())
            REQUIRE(std::memcmp(reference_host.data(),
                                fused_host.data(),
                                reference_host.size() * sizeof(Float))
                    == 0);
        for(Float energy : fused_host)
        {
            REQUIRE(std::isfinite(energy));
            REQUIRE(energy >= Float{0.0});
        }
    }

    IPCSimplexFrictionalContactEnergyLaunchInfo make_info(DeviceBuffer<Float>& pt_energy,
                                                          DeviceBuffer<Float>& ee_energy,
                                                          DeviceBuffer<Float>& pe_energy,
                                                          DeviceBuffer<Float>& pp_energy)
    {
        return IPCSimplexFrictionalContactEnergyLaunchInfo{
            .contact_tabular     = contact_tabular.view(),
            .contact_element_ids = d_contact_ids.view(),
            .positions           = d_positions.view(),
            .prev_positions      = d_prev_positions.view(),
            .rest_positions      = d_rest_positions.view(),
            .thicknesses         = d_thicknesses.view(),
            .d_hats              = d_d_hats.view(),
            .PTs                 = d_PTs.view(),
            .EEs                 = d_EEs.view(),
            .PEs                 = d_PEs.view(),
            .PPs                 = d_PPs.view(),
            .PT_energies         = pt_energy.view(),
            .EE_energies         = ee_energy.view(),
            .PE_energies         = pe_energy.view(),
            .PP_energies         = pp_energy.view(),
            .lagged              = d_lagged.view(),
            .eps_velocity        = Float{0.2},
            .dt                  = Float{0.1}};
    }

    DeviceBuffer2D<ContactCoeff>           contact_tabular;
    DeviceBuffer<IndexT>                   d_contact_ids;
    DeviceBuffer<Vector3>                  d_positions;
    DeviceBuffer<Vector3>                  d_prev_positions;
    DeviceBuffer<Vector3>                  d_rest_positions;
    DeviceBuffer<Float>                    d_thicknesses;
    DeviceBuffer<Float>                    d_d_hats;
    std::vector<Vector4i>                  PTs;
    std::vector<Vector4i>                  EEs;
    std::vector<Vector3i>                  PEs;
    std::vector<Vector2i>                  PPs;
    DeviceBuffer<Vector4i>                 d_PTs;
    DeviceBuffer<Vector4i>                 d_EEs;
    DeviceBuffer<Vector3i>                 d_PEs;
    DeviceBuffer<Vector2i>                 d_PPs;
    DeviceBuffer<Float>                    reference_PT;
    DeviceBuffer<Float>                    reference_EE;
    DeviceBuffer<Float>                    reference_PE;
    DeviceBuffer<Float>                    reference_PP;
    DeviceBuffer<Float>                    fused_PT;
    DeviceBuffer<Float>                    fused_EE;
    DeviceBuffer<Float>                    fused_PE;
    DeviceBuffer<Float>                    fused_PP;
    DeviceBuffer<IPCSimplexFrictionLagged> d_lagged;
};
}  // namespace

TEST_CASE("IPC simplex frictional-contact four-launch oracle matches fused energy",
          "[cuda][line_search][friction_energy_fusion]")
{
    EnergyFixture fixture;

    SECTION("all contact classes empty")
    {
        fixture.compare(Counts{});
    }
    SECTION("each contact class alone")
    {
        fixture.compare(Counts{.pt = 3});
        fixture.compare(Counts{.ee = 3});
        fixture.compare(Counts{.pe = 3});
        fixture.compare(Counts{.pp = 3});
    }
    SECTION("mixed ranges cross every type boundary")
    {
        fixture.compare(Counts{.pt = 2, .ee = 3, .pe = 4, .pp = 5});
    }
    SECTION("empty interior ranges preserve cumulative offsets")
    {
        fixture.compare(Counts{.pt = 2, .pe = 3});
        fixture.compare(Counts{.ee = 2, .pp = 3});
    }
    SECTION("CTA boundaries repeat valid contact geometry")
    {
        for(int count : std::array{127, 128, 129, 255, 256, 257, 511, 512, 513})
        {
            CAPTURE(count);
            fixture.compare(Counts{.pt = count});
            fixture.compare(Counts{.ee = count});
            fixture.compare(Counts{.pe = count});
            fixture.compare(Counts{.pp = count});
        }
        // At CTA256, type boundaries straddle 256 and the final PP entry
        // starts the third block.
        fixture.compare(Counts{.pt = 255, .ee = 1, .pe = 1, .pp = 256});
    }
    SECTION("reused capacities survive nonzero zero nonzero")
    {
        fixture.compare(Counts{.pt = 5, .ee = 4, .pe = 3, .pp = 2});
        fixture.compare(Counts{});
        fixture.compare(Counts{.pt = 1, .ee = 2, .pe = 3, .pp = 4});
    }
}
