/*
 * NVIDIA contributions: interfaces for candidate-reuse bounds, batched copies
 * and step forward, read-back buffers and the per-axis maximum.
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
#include <cuda_tool/cuda_tool.h>
#include <cuda_tool/copy_batch.h>
#include <functional>
#include <Eigen/Geometry>
#include <collision_detection/aabb.h>
#include <utils/dump_utils.h>
#include <utils/offset_count_collection.h>

namespace uipc::backend::cuda
{
class GlobalTrajectoryFilter;
class GlobalActiveSetManager;
class StepBatch;
class GlobalJointDofManager;
class VertexReporter;
class GlobalVertexManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class VertexCountInfo
    {
      public:
        void count(SizeT count) noexcept;
        void changeable(bool is_changable) noexcept;

      private:
        friend class GlobalVertexManager;
        friend class VertexReporter;

        SizeT m_count     = 0;
        bool  m_changable = false;
    };

    class VertexAttributeInfo
    {
      public:
        VertexAttributeInfo(Impl* impl, SizeT index, SizeT frame) noexcept;
        SizeT                          frame() const noexcept;
        cuda_tool::BufferView<Vector3> rest_positions() const noexcept;
        cuda_tool::BufferView<Float>   thicknesses() const noexcept;
        cuda_tool::BufferView<IndexT>  coindices() const noexcept;
        cuda_tool::BufferView<IndexT>  dimensions() const noexcept;
        cuda_tool::BufferView<Vector3> positions() const noexcept;
        cuda_tool::BufferView<IndexT>  contact_element_ids() const noexcept;
        cuda_tool::BufferView<IndexT>  subscene_element_ids() const noexcept;
        cuda_tool::BufferView<IndexT>  body_ids() const noexcept;
        // vert-wise d_hat
        cuda_tool::BufferView<Float> d_hats() const noexcept;

        /**
         * @breif require discard friction, if this update will ruin the friction computation
         *
         */
        void require_discard_friction() const noexcept;

      private:
        friend class GlobalVertexManager;
        SizeT m_index;
        Impl* m_impl;
        SizeT m_frame;
    };

    class VertexDisplacementInfo
    {
      public:
        VertexDisplacementInfo(Impl* impl, SizeT index, cuda_tool::CopyBatch* copies = nullptr) noexcept;
        cuda_tool::BufferView<Vector3> displacements() const noexcept;
        cuda_tool::CBufferView<IndexT> coindices() const noexcept;
        // displacements() = src, or every displacement zero. Both join the
        // batched copies of the line-search start point when the engine
        // collects the displacements with them, and run at once otherwise.
        void copy_displacements_from(cuda_tool::CBufferView<Vector3> src) const;
        void zero_displacements() const;

        // A reporter that writes its displacements with its own kernel can
        // take their per-axis maximum |d| there (axis_max_add() in
        // global_geometry/axis_max.h, every displacement of its range): when
        // every reporter did so or zeroed its range,
        // compute_axis_max_displacement() reads that maximum instead of
        // reducing the displacements (the same value: a maximum does not
        // depend on the order). Calling this commits the reporter to it.
        struct AxisMax
        {
            unsigned long long* slot  = nullptr;
            unsigned long long* clear = nullptr;
        };
        AxisMax axis_max() const noexcept;

      private:
        friend class GlobalVertexManager;
        SizeT                 m_index;
        Impl*                 m_impl;
        cuda_tool::CopyBatch* m_copies = nullptr;
    };

    /**
     * @brief A mapping from the global vertex index to the coindices.
     * 
     * The values of coindices is dependent on the reporters, which can be:
     * 1) the local index of the vertex
     * 2) or any other information that is needed to be stored.
     */
    cuda_tool::CBufferView<IndexT> coindices() const noexcept;

    /**
     * @brief A mapping from the global vertex index to the body id.
     */
    cuda_tool::CBufferView<IndexT> body_ids() const noexcept;

    /**
     * @brief The d_hat of the vertices.
     * 
     * The d_hat is used to compute the penetration depth.
     */
    cuda_tool::CBufferView<Float> d_hats() const noexcept;

    /**
     * @brief The current positions of the vertices.
     */
    cuda_tool::CBufferView<Vector3> positions() const noexcept;

    /**
     * @brief The positions of the vertices at last time step.
     * 
     * Used to compute the friction.
     */
    cuda_tool::CBufferView<Vector3> prev_positions() const noexcept;

    /**
     * @brief The rest positions of the vertices.
     * 
     * Can be used to retrieve some quantities at the rest state.
     */
    cuda_tool::CBufferView<Vector3> rest_positions() const noexcept;

    /**
     * @brief The safe positions of the vertices in line search.
     *  
     * Used as a start point to do the line search.
     */
    cuda_tool::CBufferView<Vector3> safe_positions() const noexcept;

    /**
     * @brief Indicate the contact element id of the vertices.
     */
    cuda_tool::CBufferView<IndexT> contact_element_ids() const noexcept;

    /**
     * @brief Indicate the contact element id of the vertices.
     */
    cuda_tool::CBufferView<IndexT> subscene_element_ids() const noexcept;

    /**
     * @brief The displacements of the vertices (after solving the linear system).
     * 
     * The displacements are not scaled by the alpha.
     */
    cuda_tool::CBufferView<Vector3> displacements() const noexcept;

    /**
     * @brief The thicknesses of the vertices.
     * 
     * The thicknesses are used to compute the penetration depth.
     */
    cuda_tool::CBufferView<Float> thicknesses() const noexcept;

    /**
     * @brief The dimension of the vertices. 
     * - 0: Codim 0D
     * - 1: Codim 1D
     * - 2: Codim 2D
     * - 3: 3D
     */
    cuda_tool::CBufferView<IndexT> dimensions() const noexcept;

    /**
     * @brief the axis align bounding box of the all vertices.
     */
    AABB vertex_bounding_box() const noexcept;

    /**
     * @brief the diagonal length of the rest-position bounding box, computed
     * once at init. Used for scene-relative parameter adaptation (e.g.
     * contact/d_hat_relative, newton/velocity_tol_relative).
     */
    Float scene_diagonal() const noexcept;

  public:
    class Impl
    {
      public:
        Impl() = default;
        void init();
        void update_attributes(SizeT frame);
        void rebuild();

        void record_prev_positions();
        void record_start_point(cuda_tool::CopyBatch& copies);
        void step_forward(Float alpha);
        void step_forward(StepBatch& steps);

        void collect_vertex_displacements(cuda_tool::CopyBatch* copies);

        void setup_ccd(cuda_tool::CBufferView<Vector3> base_positions);
        void restore_ccd();
        void overwrite_positions(cuda_tool::CBufferView<Vector3> src);

        Float compute_axis_max_displacement();
        void  launch_axis_max_displacement(cuda_tool::VarView<Float> max_disp,
                                           cudaStream_t              stream);
        AABB  compute_vertex_bounding_box();

        template <typename T>
        cuda_tool::BufferView<T> subview(cuda_tool::DeviceBuffer<T>& buffer,
                                         SizeT index) const noexcept;

        bool dump(DumpInfo& info);
        bool try_recover(RecoverInfo& info);
        void apply_recover(RecoverInfo& info);
        void clear_recover(RecoverInfo& info);

        Float default_d_hat = 0.01;
        Float d_hat_relative = 0.0;  // > 0: override default with relative * scene_diagonal (Stiff-GIPC convention)

        cuda_tool::DeviceBuffer<IndexT>  coindices;
        cuda_tool::DeviceBuffer<IndexT>  body_ids;
        cuda_tool::DeviceBuffer<Float>   d_hats;
        cuda_tool::DeviceBuffer<IndexT>  dimensions;
        cuda_tool::DeviceBuffer<Vector3> positions;
        cuda_tool::DeviceBuffer<Vector3> prev_positions;
        cuda_tool::DeviceBuffer<Vector3> rest_positions;
        cuda_tool::DeviceBuffer<Vector3> safe_positions;
        cuda_tool::DeviceBuffer<Float>   thicknesses;
        cuda_tool::DeviceBuffer<IndexT>  contact_element_ids;
        cuda_tool::DeviceBuffer<IndexT>  subscene_element_ids;
        cuda_tool::DeviceBuffer<Vector3> displacements;
        cuda_tool::DeviceBuffer<Float>   displacement_norms;

        // compute_axis_max_displacement()'s reduction stores its result here
        // (cuda_tool::ReadbackBuffer)
        cuda_tool::ReadbackBuffer<Float> axis_max_disp;
        cuda_tool::DeviceVar<Float>      max_disp_norm;
        cuda_tool::DeviceVar<Vector3>    min_pos;
        cuda_tool::DeviceVar<Vector3>    max_pos;

        // The reporters' maxima of the current displacements
        // (VertexDisplacementInfo::axis_max()): two slots, used in turn by
        // successive collections; the first reporter of a collection clears
        // the other one for the next.
        cuda_tool::DeviceBuffer<unsigned long long> axis_max_slots;
        SizeT                                       axis_max_generation = 0;
        SizeT axis_max_covered    = 0;      // reporters that took or zeroed
        bool  axis_max_taken      = false;  // a reporter took the slot
        bool  axis_max_fused      = false;  // the slot holds the maximum
        bool  axis_max_next_clear = true;   // the next collection's slot is 0

        // compute_axis_max_displacement() of the current displacements, valid
        // until collect_vertex_displacements or setup_ccd rewrites them
        Float h_axis_max_disp       = 0.0;
        bool  h_axis_max_disp_valid = false;


        SimSystemSlot<GlobalTrajectoryFilter>   global_trajectory_filter;
        SimSystemSlot<GlobalActiveSetManager>   global_active_set_manager;
        SimSystemSlot<GlobalJointDofManager>    global_joint_dof_manager;
        SimSystemSlotCollection<VertexReporter> vertex_reporters;

        OffsetCountCollection<IndexT> reporter_vertex_offsets_counts;

        AABB vertex_bounding_box;

        // diagonal length of the rest-position bounding box, set in init()
        Float scene_diagonal = 0.0;

        BufferDump dump_positions;
        BufferDump dump_prev_positions;
    };

  protected:
    virtual void do_build() override;
    virtual bool do_dump(DumpInfo& info) override;
    virtual bool do_try_recover(RecoverInfo& info) override;
    virtual void do_apply_recover(RecoverInfo& info) override;
    virtual void do_clear_recover(RecoverInfo& info) override;

  private:
    friend class SimEngine;
    friend class MaxTranslationChecker;
    friend class GlobalTrajectoryFilter;
    friend class GlobalActiveSetManager;

    // Initialize the global vertex manager
    // - Create the surface mesh
    // - Setup surface attributes
    void init();

    // Update the surface attributes
    void update_attributes();

    void rebuild();
    void record_prev_positions();
    void collect_vertex_displacements();
    // The displacement copies join `copies` (launched by the caller).
    void  collect_vertex_displacements(cuda_tool::CopyBatch& copies);
    Float compute_axis_max_displacement();
    // The same maximum written to `max_disp` on `stream`, without a host read.
    void launch_axis_max_displacement(cuda_tool::VarView<Float> max_disp, cudaStream_t stream);

    AABB compute_vertex_bounding_box();
    void step_forward(Float alpha);
    // The position step joins `steps` (launched by the caller).
    void step_forward(StepBatch& steps);
    void record_start_point();
    // The start-point copy joins `copies` (launched by the caller).
    void record_start_point(cuda_tool::CopyBatch& copies);

    void setup_ccd(cuda_tool::CBufferView<Vector3> base_positions);
    void restore_ccd();
    void overwrite_positions(cuda_tool::CBufferView<Vector3> src);

    friend class VertexReporter;
    void add_reporter(VertexReporter* reporter);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/global_vertex_manager.inl"
