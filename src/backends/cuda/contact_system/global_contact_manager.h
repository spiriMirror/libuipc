/*
 * NVIDIA contributions: interfaces for the side-stream feasible step, the
 * contact-tabular version, the prelaunched energy extent and the read-back of
 * the feasible-step minimum through mapped memory.
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
#include <global_geometry/global_vertex_manager.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <cuda_tool/cuda_tool.h>
#include <contact_system/contact_coeff.h>
#include <algorithm/matrix_converter.h>
#include <utils/offset_count_collection.h>

namespace uipc::backend::cuda
{
class ContactReporter;
class ContactReceiver;
class GlobalTrajectoryFilter;
class GlobalSimplicialSurfaceManager;
class AdaptiveContactParameterReporter;
class FiniteElementMethod;
class AffineBodyDynamics;

class GlobalContactManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    using GradientHessianExtentInfo = GlobalDyTopoEffectManager::GradientHessianExtentInfo;

    using GradientHessianInfo = GlobalDyTopoEffectManager::GradientHessianInfo;

    using EnergyExtentInfo = GlobalDyTopoEffectManager::EnergyExtentInfo;

    using PrelaunchEnergyExtentInfo = GlobalDyTopoEffectManager::PrelaunchEnergyExtentInfo;

    using EnergyInfo = GlobalDyTopoEffectManager::EnergyInfo;

    class Impl;

    class AdaptiveParameterInfo
    {
      public:
        AdaptiveParameterInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        cuda_tool::Buffer2DView<ContactCoeff> contact_tabular() const noexcept;

        S<cuda_tool::DeviceBuffer2D<ContactCoeff>> exchange_contact_tabular(
            S<cuda_tool::DeviceBuffer2D<ContactCoeff>> new_buffer) const noexcept;

      private:
        Impl* m_impl;
    };

    class Impl
    {
      public:
        void  init(WorldVisitor& world);
        void  _build_contact_tabular(WorldVisitor& world);
        void  _build_subscene_tabular(WorldVisitor& world);
        void  compute_d_hat();
        void  compute_adaptive_kappa();
        Float compute_cfl_condition();
        void  launch_feasible_step(cudaStream_t stream);
        Float finish_feasible_step();
        Float compute_feasible_step();

        SimSystemSlot<GlobalVertexManager>            global_vertex_manager;
        SimSystemSlot<GlobalTrajectoryFilter>         global_trajectory_filter;
        SimSystemSlot<GlobalSimplicialSurfaceManager> global_surface_manager;
        SimSystemSlot<FiniteElementMethod>            finite_element_method;
        SimSystemSlot<AffineBodyDynamics>             affine_body_dynamics;

        bool cfl_enabled = false;

        S<cuda_tool::DeviceBuffer2D<ContactCoeff>> contact_tabular;
        // Incremented whenever the contact tabular is exchanged or adapted.
        SizeT contact_tabular_version = 0;

        vector<ContactCoeff>              h_contact_tabular;
        vector<IndexT>                    h_contact_mask_tabular;
        vector<IndexT>                    h_subcene_mask_tabular;
        cuda_tool::DeviceBuffer2D<IndexT> contact_mask_tabular;
        cuda_tool::DeviceBuffer2D<IndexT> subscene_mask_tabular;
        Float                             reserve_ratio = 1.1;

        Float                                   d_hat = 0.0;
        Float                                   kappa = 0.0;
        S<const geometry::AttributeSlot<Float>> dt_attr;
        Float                                   eps_velocity = 0.0;

        // Scene-adaptive kappa corridor, ported from Stiff-GIPC's
        // suggestKappa/upperBoundKappa (GIPC.cu:9850-9888) with the dt^2
        // convention conversion. Computed once in init(); <= 0 means "not
        // computable for this scene" and callers fall back to the
        // contact/adaptive/{min,max}_kappa config values.
        Float kappa_lower_bound = -1.0;
        Float kappa_upper_bound = -1.0;


        /***********************************************************************
        *                     Global Vertex Contact Info                       *
        ***********************************************************************/

        cuda_tool::DeviceBuffer<IndexT> vert_is_active_contact;
        cuda_tool::DeviceBuffer<Float>  vert_disp_norms;
        cuda_tool::DeviceVar<Float>     max_disp_norm;

        cuda_tool::DeviceBuffer<Float> feasible_tois;
        // the reduction stores the minimum straight into the read-back
        // buffer (no copy follows it on the mapped path)
        cuda_tool::ReadbackBuffer<Float> min_feasible_toi;
        cudaStream_t                     feasible_stream  = nullptr;
        bool                             feasible_pending = false;

        SimSystemSlotCollection<ContactReporter> contact_reporters;
        SimSystemSlotCollection<ContactReceiver> contact_receivers;
        SimSystemSlot<AdaptiveContactParameterReporter> adaptive_contact_parameter_reporter;
    };

    Float d_hat() const;
    Float eps_velocity() const;
    bool  cfl_enabled() const;

    // Scene-adaptive kappa corridor (Stiff-GIPC suggestKappa/upperBoundKappa
    // with the dt^2 conversion); <= 0 = not computable, use config fallback
    Float kappa_lower_bound() const;
    Float kappa_upper_bound() const;

    cuda_tool::CBuffer2DView<ContactCoeff> contact_tabular() const noexcept;
    // Changes whenever the contact tabular is exchanged or its coefficients
    // are adapted (the adaptive kappa evaluates the contacts with a test
    // tabular, then writes the new kappa).
    SizeT                            contact_tabular_version() const noexcept;
    cuda_tool::CBuffer2DView<IndexT> contact_mask_tabular() const noexcept;
    cuda_tool::CBuffer2DView<IndexT> subscene_mask_tabular() const noexcept;


  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    friend class ContactLineSearchReporter;
    friend class GlobalTrajectoryFilter;
    friend class ContactExporterManager;

    void init();

    void compute_adaptive_parameters();

    Float compute_cfl_condition();

    // Stiff-GIPC-style feasible-step pre-cap: over the current active contact
    // set, the largest Newton step that keeps every pair's gap >= (1-slackness)
    // of its current value. Applied before trajectory candidate generation to
    // shrink the swept-AABB candidate set.
    Float compute_feasible_step();
    // The same in two halves: launch_feasible_step() enqueues all of its
    // work, including the copy of the result to the host, on `stream`;
    // finish_feasible_step() waits for that stream and returns the step.
    void  launch_feasible_step(cudaStream_t stream);
    Float finish_feasible_step();


    friend class ContactReporter;
    void add_reporter(ContactReporter* reporter);

    friend class AdaptiveContactParameterReporter;
    void add_reporter(AdaptiveContactParameterReporter* reporter);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda