/*
 * NVIDIA contributions: interfaces for raw-entry distribution, side-stream
 * assembly and batched count read-backs; for the packed distribution.
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
#include <energy_component_flags.h>
#include <global_geometry/global_vertex_manager.h>
#include <cuda_tool/cuda_tool.h>
#include <utils/offset_count_collection.h>
#include <algorithm/matrix_converter.h>
#include <dytopo_effect_system/dytopo_classify_info.h>

namespace uipc::backend::cuda
{
class DyTopoEffectReporter;
class DyTopoEffectReceiver;

class GlobalDyTopoEffectManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    ~GlobalDyTopoEffectManager() override;

    class Impl;

    class GradientHessianExtentInfo
    {
      public:
        bool gradient_only() const { return m_gradient_only; }
        void gradient_count(SizeT count) noexcept { m_gradient_count = count; }
        void hessian_count(SizeT count) noexcept { m_hessian_count = count; }

      private:
        friend class Impl;
        friend class DyTopoEffectReporter;

        bool  m_gradient_only  = false;
        SizeT m_gradient_count = 0;
        SizeT m_hessian_count  = 0;
    };

    class GradientHessianInfo
    {
      public:
        bool gradient_only() const noexcept { return m_gradient_only; }
        cuda_tool::DoubletVectorView<Float, 3> gradients() const noexcept
        {
            return m_gradients;
        }
        cuda_tool::TripletMatrixView<Float, 3> hessians() const noexcept
        {
            return m_hessians;
        }
        // Stream for every launch of this reporter's assembly. Reporters
        // write disjoint ranges, so each runs on its own side stream; one
        // that keeps the legacy stream stays correct but runs serially.
        cudaStream_t stream() const noexcept { return m_stream; }


      private:
        friend class Impl;
        bool                                   m_gradient_only = false;
        cuda_tool::DoubletVectorView<Float, 3> m_gradients;
        cuda_tool::TripletMatrixView<Float, 3> m_hessians;
        cudaStream_t                           m_stream = nullptr;
    };

    class EnergyExtentInfo
    {
      public:
        void energy_count(SizeT count) noexcept { m_energy_count = count; }

      private:
        friend class Impl;
        friend class DyTopoEffectLineSearchReporter;
        SizeT m_energy_count = 0;
    };

    class EnergyInfo
    {
      public:
        cuda_tool::BufferView<Float> energies() const { return m_energies; }
        bool                         is_initial() const { return m_is_initial; }

      private:
        friend class DyTopoEffectLineSearchReporter;
        cuda_tool::BufferView<Float> m_energies;
        bool                         m_is_initial = false;
    };

    using ClassifyInfo = DyTopoClassifyInfo;

    // The raw assembled entries in the receiver's range, unsorted: the same
    // vertex i or block (i, j) may appear several times, and a receiver must
    // accumulate them (for example atomic gradient adds, or Hessian triplets
    // that the global linear system's conversion sums).
    class ClassifiedDyTopoEffectInfo
    {
      public:
        cuda_tool::CDoubletVectorView<Float, 3> gradients() const noexcept
        {
            return m_gradients;
        }
        cuda_tool::CTripletMatrixView<Float, 3> hessians() const noexcept
        {
            return m_hessians;
        }

      private:
        friend class Impl;
        cuda_tool::CDoubletVectorView<Float, 3> m_gradients;
        cuda_tool::CTripletMatrixView<Float, 3> m_hessians;
    };

    class ComputeDyTopoEffectInfo
    {
      public:
        void gradient_only(bool v) noexcept { m_gradient_only = v; }
        void component_flags(EnergyComponentFlags v) noexcept
        {
            m_component_flags = v;
        }

      private:
        friend class Impl;
        bool                 m_gradient_only   = false;
        EnergyComponentFlags m_component_flags = EnergyComponentFlags::All;
    };

    class Impl
    {
      public:
        void init(WorldVisitor& world);
        void compute_dytopo_effect(ComputeDyTopoEffectInfo& info);
        void _assemble(ComputeDyTopoEffectInfo& info);
        void _distribute(ComputeDyTopoEffectInfo& info);

        SimSystemSlot<GlobalVertexManager> global_vertex_manager;

        Float reserve_ratio = 1.1;


        /***********************************************************************
        *                              Reporter                                *
        ***********************************************************************/

        SimSystemSlotCollection<DyTopoEffectReporter> dytopo_effect_reporters;
        SimSystemSlotCollection<DyTopoEffectReporter> contact_reporters;
        SimSystemSlotCollection<DyTopoEffectReporter> non_contact_reporters;

        OffsetCountCollection<IndexT> reporter_energy_offsets_counts;
        OffsetCountCollection<IndexT> reporter_gradient_offsets_counts;
        OffsetCountCollection<IndexT> reporter_hessian_offsets_counts;

        cuda_tool::DeviceTripletMatrix<Float, 3> collected_dytopo_effect_hessian;
        cuda_tool::DeviceDoubletVector<Float, 3> collected_dytopo_effect_gradient;

        // One side stream per reporter (up to a few): the reporters' assembly
        // kernels write disjoint ranges of the collected buffers and overlap.
        cuda_tool::SideStreams assembly_streams;
        // the packed gradient selection (on a side stream) is complete
        cudaEvent_t gradient_selected = nullptr;

        /***********************************************************************
        *                               Receiver                               *
        ***********************************************************************/

        SimSystemSlotCollection<DyTopoEffectReceiver> dytopo_effect_receivers;

        // Selection flags and their exclusive scans (one trailing element
        // for the count), per receiver, so that every receiver's counts come
        // back in one host synchronization.
        struct ReceiverSelection
        {
            cuda_tool::DeviceBuffer<IndexT> gradient;
            cuda_tool::DeviceBuffer<IndexT> gradient_offsets;
            cuda_tool::DeviceBuffer<IndexT> hessian;
            cuda_tool::DeviceBuffer<IndexT> hessian_offsets;
        };
        vector<ReceiverSelection> receiver_selections;
        // packed selections of every receiver (_distribute_packed())
        cuda_tool::DeviceBuffer<uint64_t> packed_gradient;
        cuda_tool::DeviceBuffer<uint64_t> packed_gradient_offsets;
        cuda_tool::DeviceBuffer<uint64_t> packed_hessian;
        cuda_tool::DeviceBuffer<uint64_t> packed_hessian_offsets;
        bool _distribute_packed(ComputeDyTopoEffectInfo& info);
        vector<DyTopoClassifyInfo> receiver_classify_infos;
        // gradient and Hessian count of receiver i at 2i and 2i + 1, stored
        // by one gathering kernel (a read-back buffer: no copies on the
        // mapped path)
        cuda_tool::ReadbackBuffer<IndexT> receiver_selected_counts;

        vector<cuda_tool::DeviceTripletMatrix<Float, 3>> classified_dytopo_effect_hessians;
        vector<cuda_tool::DeviceDoubletVector<Float, 3>> classified_dytopo_effect_gradients;

        void loose_resize_entries(cuda_tool::DeviceTripletMatrix<Float, 3>& m, SizeT size);
        void loose_resize_entries(cuda_tool::DeviceDoubletVector<Float, 3>& v, SizeT size);
        template <typename T>
        void loose_resize(cuda_tool::DeviceBuffer<T>& buffer, SizeT size)
        {
            if(size > buffer.capacity())
            {
                buffer.reserve_discard(size * reserve_ratio);
            }
            buffer.resize_discard(size);
        }
    };

    void compute_dytopo_effect(ComputeDyTopoEffectInfo& info);

  protected:
    virtual void do_build() override;

  private:
    friend class DyTopoEffectLineSearchReporter;
    void init();

    friend class SimEngine;
    // only be called by SimEngine
    void compute_dytopo_effect();

    friend class DyTopoEffectReporter;
    void add_reporter(DyTopoEffectReporter* reporter);
    friend class DyTopoEffectReceiver;
    void add_receiver(DyTopoEffectReceiver* receiver);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
