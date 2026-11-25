/*
 * NVIDIA contributions: interfaces for side-stream energy producers and the
 * early launch of positional energies.
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
#include <line_search/line_search_reporter.h>
#include <affine_body/affine_body_dynamics.h>
#include <utils/offset_count_collection.h>


namespace uipc::backend::cuda
{
class ABDLineSearchSubreporter;
class ABDLineSearchReporter final : public LineSearchReporter
{
  public:
    using LineSearchReporter::LineSearchReporter;

    class ReportExtentInfo
    {
      public:
        void energy_count(SizeT count) { m_energy_count = count; }

      private:
        friend class ABDLineSearchReporter;
        SizeT m_energy_count = 0;
    };

    class ComputeEnergyInfo
    {
      public:
        explicit ComputeEnergyInfo(cuda_tool::SideStreams* side_streams = nullptr) noexcept
            : m_side_streams(side_streams)
            , m_stream(side_streams)
        {
        }

        cuda_tool::BufferView<Float> energies() const { return m_energies; }
        Float                        dt() const noexcept { return m_dt; }
        // Every launch of the energy producer goes to this stream.
        cudaStream_t stream() const noexcept { return m_stream.get(); }
        // For a producer that hands a stream to each of its own producers.
        cuda_tool::SideStreams* side_streams() const noexcept
        {
            return m_side_streams;
        }

      private:
        friend class ABDLineSearchReporter;
        cuda_tool::BufferView<Float> m_energies;
        Float                        m_dt           = 0.0;
        cuda_tool::SideStreams*      m_side_streams = nullptr;
        cuda_tool::LazySideStream    m_stream;
    };

    class Impl
    {
      public:
        void init(LineSearchReporter::InitInfo& info);
        void record_start_point(LineSearcher::RecordInfo& info);
        void step_forward(LineSearcher::StepInfo& info);
        void compute_energy(LineSearcher::ComputeEnergyInfo& info);
        void reduce_energy(LineSearcher::ComputeEnergyInfo& info);

        SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;

        // intermediate energy buffers
        cuda_tool::DeviceBuffer<Float> body_id_to_kinetic_energy;
        cuda_tool::DeviceVar<Float>    abd_kinetic_energy;
        cuda_tool::DeviceBuffer<Float> body_id_to_shape_energy;
        cuda_tool::DeviceVar<Float>    abd_shape_energy;

        SimSystemSlotCollection<ABDLineSearchSubreporter> reporters;
        OffsetCountCollection<IndexT>  reporter_energy_offsets_counts;
        cuda_tool::DeviceBuffer<Float> reporter_energies;
        cuda_tool::DeviceVar<Float>    total_reporter_energy;

        AffineBodyDynamics::Impl& abd() const
        {
            return affine_body_dynamics->m_impl;
        }
    };

  private:
    virtual void do_build(LineSearchReporter::BuildInfo& info) override;
    virtual void do_init(LineSearchReporter::InitInfo& info) override;
    virtual void do_record_start_point(LineSearcher::RecordInfo& info) override;
    virtual void do_step_forward(LineSearcher::StepInfo& info) override;
    virtual void do_compute_energy(LineSearcher::ComputeEnergyInfo& info) override;
    virtual void do_reduce_energy(LineSearcher::ComputeEnergyInfo& info) override;
    // The energies read the stepped DOFs only.
    virtual bool do_reads_active_set() const noexcept override { return false; }

    friend class ABDLineSearchSubreporter;
    void add_reporter(ABDLineSearchSubreporter* reporter);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda