/*
 * NVIDIA contributions: interfaces and state of the prelaunched and in-place
 * FEM assembly, its split around the contact launch and the producers' grid
 * cap.
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
#include <algorithm/matrix_converter.h>
#include <linear_system/diag_linear_subsystem.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_vertex_reporter.h>
#include <utils/offset_count_collection.h>

namespace uipc::backend::cuda
{
class FiniteElementKinetic;
class FEMDyTopoEffectReceiver;
class FEMLinearSubsystemReporter;
class FEMLinearSubsystem final : public DiagLinearSubsystem
{
  public:
    using DiagLinearSubsystem::DiagLinearSubsystem;
    ~FEMLinearSubsystem() override;

    class ComputeGradientHessianInfo
    {
      public:
        ComputeGradientHessianInfo(bool gradient_only,
                                   cuda_tool::DoubletVectorView<Float, 3> gradients,
                                   cuda_tool::TripletMatrixView<Float, 3, 3> hessians,
                                   Float        dt,
                                   cudaStream_t stream = nullptr) noexcept
            : m_gradient_only(gradient_only)
            , m_gradients(gradients)
            , m_hessians(hessians)
            , m_dt(dt)
            , m_stream(stream)
        {
        }

        auto gradient_only() const noexcept { return m_gradient_only; }
        auto gradients() const noexcept { return m_gradients; }
        auto hessians() const noexcept { return m_hessians; }
        auto dt() const noexcept { return m_dt; }
        // Stream for every launch of the kinetic gradient and Hessian.
        cudaStream_t stream() const noexcept { return m_stream; }

      private:
        bool                                      m_gradient_only = false;
        cuda_tool::DoubletVectorView<Float, 3>    m_gradients;
        cuda_tool::TripletMatrixView<Float, 3, 3> m_hessians;
        Float                                     m_dt     = 0.0;
        cudaStream_t                              m_stream = nullptr;
    };

    class ReportExtentInfo
    {
      public:
        // DoubletVector3 count
        void gradient_count(SizeT size);
        // TripletMatrix3x3 count
        void hessian_count(SizeT size);
        bool gradient_only() const noexcept
        {
            m_gradient_only_checked = true;
            return m_gradient_only;
        }
        void check(std::string_view name) const;

      private:
        friend class FEMLinearSubsystem;
        friend class FEMLinearSubsystemReporter;
        SizeT        m_gradient_count        = 0;
        SizeT        m_hessian_count         = 0;
        bool         m_gradient_only         = false;
        mutable bool m_gradient_only_checked = false;
    };

    class Impl;

    class AssembleInfo
    {
      public:
        AssembleInfo(Impl*                                     impl,
                     IndexT                                    index,
                     cuda_tool::TripletMatrixView<Float, 3, 3> hessians,
                     bool                                      gradient_only,
                     cudaStream_t stream = nullptr) noexcept
            : m_impl(impl)
            , m_index(index)
            , m_hessians(hessians)
            , m_gradient_only(gradient_only)
            , m_stream(stream)
        {
        }

        cuda_tool::DoubletVectorView<Float, 3>    gradients() const;
        cuda_tool::TripletMatrixView<Float, 3, 3> hessians() const;
        Float                                     dt() const noexcept;
        bool gradient_only() const noexcept;
        // Stream for every launch of the reporter's producers. They must
        // not launch elsewhere: in a prelaunch the Hessian slots are marked
        // unwritten on this stream first, and after an in-place prelaunch
        // (writing the global system) a fix-up on this stream follows them;
        // the prelaunch checks this (a launch into the legacy stream fails).
        cudaStream_t stream() const noexcept { return m_stream; }

        // Direct gradients (in-place prelaunch, producers launched after the
        // contact assembly only): the FEM range of the global gradient. A
        // producer may add its gradient there itself (atomically, skipping
        // fixed vertices, as the doublet addition does) instead of writing
        // its doublets, and then calls gradients_added_directly(); the
        // doublets of a reporter that does not are added after it.
        bool direct_gradients_available() const noexcept
        {
            return m_direct_gradients.size() > 0;
        }
        cuda_tool::DenseVectorView<Float> direct_gradients() const noexcept
        {
            return m_direct_gradients;
        }
        cuda_tool::CBufferView<IndexT> is_fixed() const noexcept;
        void                           gradients_added_directly() const noexcept
        {
            m_added_directly = true;
        }
        // Adds `doublets` (a part of gradients()) to direct_gradients() on
        // stream(), for a producer that writes some of its doublets.
        void add_gradients(cuda_tool::CDoubletVectorView<Float, 3> doublets) const;
        // Grid cap for the producers' kernels (0: none). An in-place
        // prelaunch issued ahead of the contact assembly holds them to the
        // SMs the contact blocks leave (GlobalLinearSystem::prepare_prelaunch()).
        int block_cap() const noexcept { return m_block_cap; }

      private:
        friend class FEMLinearSubsystem;

        Impl*                                     m_impl  = nullptr;
        IndexT                                    m_index = ~0;
        cuda_tool::TripletMatrixView<Float, 3, 3> m_hessians;
        bool                                      m_gradient_only = false;
        cudaStream_t                              m_stream        = nullptr;
        cuda_tool::DenseVectorView<Float>         m_direct_gradients;
        int                                       m_block_cap      = 0;
        mutable bool                              m_added_directly = false;
    };

    class Impl
    {
      public:
        void init();
        void report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info);
        void receive_init_dof_info(WorldVisitor& w, GlobalLinearSystem::InitDofInfo& info);

        void report_extent(GlobalLinearSystem::DiagExtentInfo& info);

        void prepare_prelaunch(GlobalLinearSystem::PrelaunchInfo& info);
        void prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info);
        void assemble(GlobalLinearSystem::DiagInfo& info);
        void _report_reporter_extents(bool gradient_only);
        void _prepare_gradient_buffers();
        // The reporters launched: all of them, or only those that launch
        // beside the contact assembly (`beside_contact`) or only the others.
        enum class ReporterSet
        {
            All,
            BesideContact,
            AfterContact
        };
        void _compute_kinetic(bool gradient_only,
                              cuda_tool::TripletMatrixView<Float, 3, 3> hessians,
                              cudaStream_t stream);
        // direct_gradients: offered to the reporters (AssembleInfo);
        // `added_directly` collects the reporters that took it; block_cap:
        // AssembleInfo::block_cap().
        void _compute_reporters(bool gradient_only,
                                cuda_tool::TripletMatrixView<Float, 3, 3> hessians,
                                cudaStream_t stream,
                                ReporterSet  set = ReporterSet::All,
                                cuda_tool::DenseVectorView<Float> direct_gradients = {},
                                vector<int>* added_directly = nullptr,
                                int          block_cap      = 0);
        void _prelaunch_in_place(GlobalLinearSystem::PrelaunchInfo& info);
        // In-place prelaunch ahead of the contact assembly, in two calls:
        // the clears and the elastic producers before the DyTopo work, the
        // light producers and the fix-up once the contact is launched.
        void _prelaunch_in_place_ahead(GlobalLinearSystem::PrelaunchInfo& info);
        void _prelaunch_in_place_rest(GlobalLinearSystem::PrelaunchInfo& info);
        // Adds the gradient doublets [offset, offset + count) of
        // reporter_gradients to `dst` on `stream`.
        void _add_reporter_gradients(cuda_tool::DenseVectorView<Float> dst,
                                     IndexT                            offset,
                                     IndexT                            count,
                                     cudaStream_t                      stream);
        void _assemble_kinetic_reporter_gradients(GlobalLinearSystem::DiagInfo& info);
        void _gather_prelaunched_hessians(IndexT& hess_offset,
                                          GlobalLinearSystem::DiagInfo& info);
        void _assemble_dytopo_effect_gradients(GlobalLinearSystem::DiagInfo& info);
        void _assemble_dytopo_effect_hessians(IndexT& hess_offset,
                                              GlobalLinearSystem::DiagInfo& info);
        void _clear_fixed_vertex_hessians(IndexT begin, GlobalLinearSystem::DiagInfo& info);


        void  accuracy_check(GlobalLinearSystem::AccuracyInfo& info);
        void  retrieve_solution(GlobalLinearSystem::SolutionInfo& info);
        Float diag_norm(GlobalLinearSystem::DiagNormInfo& info);
        Float mass_norm(GlobalLinearSystem::DiagNormInfo& info);

        SimEngine* sim_engine = nullptr;

        SimSystemSlot<FiniteElementMethod> finite_element_method;
        FiniteElementMethod::Impl&         fem() noexcept
        {
            return finite_element_method->m_impl;
        }

        SimSystemSlot<FiniteElementVertexReporter> finite_element_vertex_reporter;

        SimSystemSlot<FEMDyTopoEffectReceiver> dytopo_effect_receiver;
        SimSystemSlotCollection<FEMLinearSubsystemReporter> reporters;

        SimSystemSlot<FiniteElementKinetic> kinetic;


        S<const geometry::AttributeSlot<Float>> dt_attr;
        Float                                   reserve_ratio = 1.5;

        OffsetCountCollection<IndexT> reporter_gradient_offsets_counts;
        OffsetCountCollection<IndexT> reporter_hessian_offsets_counts;

        cuda_tool::DeviceDoubletVector<Float, 3> kinetic_gradients;
        cuda_tool::DeviceDoubletVector<Float, 3> reporter_gradients;
        cuda_tool::DeviceBuffer<Float>           diag_blocks_norm;
        cuda_tool::DeviceVar<Float>              reduced_diag_norm;

        // Kinetic and reporter Hessians written by prelaunch_assembly() in
        // FEM-local indices; assemble() gathers them into the global system.
        cuda_tool::DeviceTripletMatrix<Float, 3> prelaunched_hessians;
        cuda_tool::SideStream                    prelaunch_stream;
        // In-place prelaunch (GlobalLinearSystem::PrelaunchInfo): the
        // kinetic and beside-contact gradients are added on this stream,
        // forked after the contact assembly and beside the elastic producer.
        cuda_tool::SideStream prelaunch_add_stream;
        // recorded on prelaunch_stream once it cleared the FEM gradient
        // range; the legacy stream's additions to that range wait for it
        cudaEvent_t gradients_cleared = nullptr;
        bool        in_place          = false;  // this prelaunch wrote in place
        // reporter extents last checked against the stream contract
        SizeT        stream_checked_hessians  = ~SizeT{0};
        SizeT        stream_checked_gradients = ~SizeT{0};
        cudaStream_t stream_check_stream      = nullptr;
        // between _prelaunch_in_place_ahead() and _prelaunch_in_place_rest()
        bool                                  ahead_pending = false;
        bool                                  ahead_check   = false;
        cudaStream_t                          ahead_stream  = nullptr;
        cudaStream_t                          ahead_light   = nullptr;
        GlobalLinearSystem::TripletMatrixView ahead_hessians;
        GlobalLinearSystem::DenseVectorView   ahead_gradients;
        vector<int>                           ahead_added_directly;
        // The gather's grid cap: one block per SM (init()).
        int   sm_count                   = 1;
        bool  prelaunched                = false;
        SizeT prelaunched_gradient_count = 0;
        SizeT prelaunched_hessian_count  = 0;

        void loose_resize_entries(cuda_tool::DeviceDoubletVector<Float, 3>& v, SizeT size);
    };

  private:
    virtual void do_build(DiagLinearSubsystem::BuildInfo& info) override;
    virtual void do_init(DiagLinearSubsystem::InitInfo& info) override;
    virtual void do_report_extent(GlobalLinearSystem::DiagExtentInfo& info) override;
    virtual void do_prepare_prelaunch(GlobalLinearSystem::PrelaunchInfo& info) override;
    virtual void do_prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info) override;
    virtual void do_assemble(GlobalLinearSystem::DiagInfo& info) override;
    virtual void do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info) override;
    virtual void do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info) override;
    virtual Float do_diag_norm(GlobalLinearSystem::DiagNormInfo& info) override;
    virtual Float do_mass_norm(GlobalLinearSystem::DiagNormInfo& info) override;
    virtual void do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info) override;
    virtual void do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info) override;
    virtual U64 get_uid() const noexcept override;

    friend class FEMLinearSubsystemReporter;
    void add_reporter(FEMLinearSubsystemReporter* reporter);

    friend class FiniteElementKinetic;
    void add_kinetic(FiniteElementKinetic* kinetic);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
