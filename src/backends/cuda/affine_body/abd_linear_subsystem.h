/*
 * NVIDIA contributions: interfaces supporting the blockwise ABD assembly path,
 * the side-stream and prelaunched producer assembly and the batched count
 * read-backs.
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
#include <linear_system/diag_linear_subsystem.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/abd_dytopo_effect_receiver.h>
#include <affine_body/affine_body_vertex_reporter.h>
#include <utils/offset_count_collection.h>

#include <cstdint>

namespace uipc::backend::cuda
{
class ABDLinearSubsystemReporter;
class ABDLinearSubsystem final : public DiagLinearSubsystem
{
  public:
    using DiagLinearSubsystem::DiagLinearSubsystem;

    cuda_tool::CBufferView<Matrix12x12> diag_hessian() const noexcept
    {
        return m_impl.diag_hessian.view();
    }

    class ComputeGradientHessianInfo
    {
      public:
        ComputeGradientHessianInfo(bool gradient_only,
                                   cuda_tool::BufferView<Vector12>    gradient,
                                   cuda_tool::BufferView<Matrix12x12> hessians,
                                   Float                              dt,
                                   cudaStream_t stream = nullptr) noexcept
            : m_gradient_only(gradient_only)
            , m_gradients(gradient)
            , m_hessians(hessians)
            , m_dt(dt)
            , m_stream(stream)
        {
        }

        auto gradient_only() const noexcept { return m_gradient_only; }
        auto hessians() const noexcept { return m_hessians; }
        auto gradients() const noexcept { return m_gradients; }
        auto dt() const noexcept { return m_dt; }
        // Stream for every launch of the kinetic/shape gradient and Hessian.
        cudaStream_t stream() const noexcept { return m_stream; }

      private:
        bool                               m_gradient_only = false;
        cuda_tool::BufferView<Matrix12x12> m_hessians;
        cuda_tool::BufferView<Vector12>    m_gradients;
        Float                              m_dt     = 0.0;
        cudaStream_t                       m_stream = nullptr;
    };

    class ReportExtentInfo
    {
      public:
        // DoubletVector12 count
        void gradient_count(SizeT size);
        // TripletMatrix12x12 count
        void hessian_count(SizeT size);
        bool gradient_only() const noexcept
        {
            m_gradient_only_checked = true;
            return m_gradient_only;
        }
        void check(std::string_view name) const;

      private:
        friend class ABDLinearSubsystem;
        friend class ABDLinearSubsystemReporter;
        SizeT        m_gradient_count        = 0;
        SizeT        m_hessian_count         = 0;
        bool         m_gradient_only         = false;
        mutable bool m_gradient_only_checked = false;
    };

    class Impl;

    class AssembleInfo
    {
      public:
        AssembleInfo(Impl*                   impl,
                     IndexT                  index,
                     bool                    gradient_only,
                     cuda_tool::SideStreams* side_streams = nullptr) noexcept;
        cuda_tool::DoubletVectorView<Float, 12>     gradients() const;
        cuda_tool::TripletMatrixView<Float, 12, 12> hessians() const;
        bool gradient_only() const noexcept;
        // Side streams for the reporter's independent producers (nullptr:
        // legacy default stream). Each producer must keep all of its
        // launches on one stream, e.g. through cuda_tool::LazySideStream.
        cuda_tool::SideStreams* side_streams() const noexcept;

      private:
        friend class ABDLinearSubsystem;

        Impl*                   m_impl          = nullptr;
        IndexT                  m_index         = ~0;
        bool                    m_gradient_only = false;
        cuda_tool::SideStreams* m_side_streams  = nullptr;
    };

    class Impl
    {
      public:
        void init();
        void report_extent(GlobalLinearSystem::DiagExtentInfo& info);

        void report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info);
        void receive_init_dof_info(WorldVisitor& w, GlobalLinearSystem::InitDofInfo& info);

        void prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info);
        void assemble(GlobalLinearSystem::DiagInfo& info);
        void _report_reporter_extents(bool gradient_only);
        void _prepare_reporter_buffers();
        void _compute_kinetic_shape(bool gradient_only, cudaStream_t stream);
        void _assemble_kinetic_shape(IndexT&                       offset,
                                     GlobalLinearSystem::DiagInfo& info,
                                     cudaStream_t                  stream);
        void _launch_reporters(bool gradient_only, cuda_tool::SideStreams* side_streams);
        void  _assemble_reporters(IndexT&                       offset,
                                  GlobalLinearSystem::DiagInfo& info,
                                  cudaStream_t                  stream);
        void  _assemble_dytopo_effect(IndexT&                       offset,
                                      GlobalLinearSystem::DiagInfo& info,
                                      cudaStream_t                  stream);
        SizeT _prepare_dytopo_body_pair_aggregation();
        // reads back the two totals and sizes the run arrays
        SizeT _finish_dytopo_body_pair_aggregation();

        void  accuracy_check(GlobalLinearSystem::AccuracyInfo& info);
        void  retrieve_solution(GlobalLinearSystem::SolutionInfo& info);
        Float diag_norm();
        Float mass_norm();

        SimSystemSlot<AffineBodyDynamics>       affine_body_dynamics;
        AffineBodyDynamics::Impl&               abd() const noexcept;
        SimSystemSlot<ABDDyTopoEffectReceiver>  dytopo_effect_receiver;
        SimSystemSlot<AffineBodyVertexReporter> affine_body_vertex_reporter;

        Float reserve_ratio = 1.5;

        SimSystemSlotCollection<ABDLinearSubsystemReporter> reporters;
        OffsetCountCollection<IndexT> reporter_gradient_offsets_counts;
        OffsetCountCollection<IndexT> reporter_hessian_offsets_counts;

        cuda_tool::DeviceTripletMatrix<Float, 12, 12> reporter_hessians;
        cuda_tool::DeviceDoubletVector<Float, 12>     reporter_gradients;
        SizeT prelaunched_gradient_count = 0;
        SizeT prelaunched_hessian_count  = 0;

        // intermediate gradient/hessian buffers for kinetic/shape
        cuda_tool::DeviceBuffer<Matrix12x12> body_id_to_shape_hessian;
        cuda_tool::DeviceBuffer<Vector12>    body_id_to_shape_gradient;
        cuda_tool::DeviceBuffer<Matrix12x12> body_id_to_kinetic_hessian;
        cuda_tool::DeviceBuffer<Vector12>    body_id_to_kinetic_gradient;

        // DyTopo receiver entries are stably grouped by the ordered body pair
        // before assembly. One aggregate then replaces all contact blocks for
        // that pair, so the global MatrixConverter never sees the 16x
        // receiver expansion.
        cuda_tool::DeviceBuffer<std::uint64_t> dytopo_body_pair_keys_input;
        cuda_tool::DeviceBuffer<std::uint64_t> dytopo_body_pair_keys;
        cuda_tool::DeviceBuffer<IndexT>        dytopo_receiver_indices_input;
        cuda_tool::DeviceBuffer<IndexT>        dytopo_receiver_indices;
        cuda_tool::DeviceBuffer<std::uint64_t> dytopo_unique_body_pair_keys;
        cuda_tool::DeviceBuffer<int>           dytopo_body_pair_counts;
        cuda_tool::DeviceBuffer<int>           dytopo_body_pair_offsets;
        cuda_tool::DeviceBuffer<int>           dytopo_body_pair_block_counts;
        cuda_tool::DeviceBuffer<int>           dytopo_body_pair_block_offsets;
        cuda_tool::DeviceVar<int> dytopo_unique_body_pair_count_dev;
        // {unique body-pair count, aggregated block count}, read back once
        cuda_tool::DeviceBuffer<int> dytopo_body_pair_count_readback;
        SizeT                        dytopo_prepared_receiver_count = 0;
        SizeT                        dytopo_unique_body_pair_count  = 0;
        SizeT                        dytopo_aggregated_block_count  = 0;
        int                          dytopo_body_pair_col_bits      = 1;

        // diag hessian for preconditioner
        cuda_tool::DeviceBuffer<Matrix12x12> diag_hessian;
        cuda_tool::DeviceBuffer<Float>       block_norm;
        cuda_tool::DeviceVar<Float>          reduced_norm;

        S<const geometry::AttributeSlot<Float>> dt_attr;
    };

  private:
    virtual void do_build(DiagLinearSubsystem::BuildInfo& info) override;
    virtual void do_init(InitInfo& info) override;

    virtual void do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info) override;
    virtual void do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info) override;

    virtual void do_report_extent(GlobalLinearSystem::DiagExtentInfo& info) override;
    virtual void do_prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info) override;
    virtual void do_assemble(GlobalLinearSystem::DiagInfo& info) override;
    virtual void do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info) override;
    virtual void do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info) override;
    virtual Float do_diag_norm(GlobalLinearSystem::DiagNormInfo& info) override;
    virtual Float do_mass_norm(GlobalLinearSystem::DiagNormInfo& info) override;

    virtual U64 get_uid() const noexcept override;

    friend class ABDLinearSubsystemReporter;
    void add_reporter(ABDLinearSubsystemReporter* reporter);  // only be called by ABDLinearSubsystemReporter

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
