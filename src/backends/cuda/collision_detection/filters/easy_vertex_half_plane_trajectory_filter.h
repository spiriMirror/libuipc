/*
 * NVIDIA contributions: interfaces for detection skins, two-phase active-set
 * filtering, the in-kernel TOI minimum and read-back buffers.
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
#include <collision_detection/vertex_half_plane_trajectory_filter.h>

namespace uipc::backend::cuda
{
class EasyVertexHalfPlaneTrajectoryFilter final : public VertexHalfPlaneTrajectoryFilter
{
  public:
    using VertexHalfPlaneTrajectoryFilter::VertexHalfPlaneTrajectoryFilter;

    class Impl
    {
      public:
        void launch_filter_active(FilterActiveInfo& info);
        void finish_filter_active(FilterActiveInfo& info);
        void filter_toi(FilterTOIInfo& info);

        void query_active(FilterActiveInfo& info);

        cuda_tool::DeviceVar<IndexT> num_collisions;
        IndexT                       h_num_collisions;
        // num_collisions, published behind the query (cuda_tool::ReadbackBuffer)
        cuda_tool::ReadbackBuffer<IndexT> num_collisions_readback;

        /**
         * @brief [Vertex-HalfPlane] pairs
         */
        cuda_tool::DeviceBuffer<Vector2i> PHs;

        Float reserve_ratio = 1.1f;

        cuda_tool::DeviceBuffer<Float> tois;
        // The TOI minimum's slot (toi_min.h): key, finished blocks.
        cuda_tool::DeviceBuffer<unsigned long long> toi_min_slot;
    };

    virtual cuda_tool::CBufferView<Vector2i> candidate_PHs() const noexcept override;
    virtual cuda_tool::CBufferView<Float> toi_PHs() const noexcept override;

  private:
    Impl m_impl;

    // Inherited via VertexHalfPlaneTrajectoryFilter

    virtual void do_build(BuildInfo& info) override;
    virtual void do_detect(DetectInfo& info) override;
    virtual void do_filter_active(FilterActiveInfo& info) override;
    virtual bool do_launch_filter_active(FilterActiveInfo& info) override;
    virtual void do_finish_filter_active(FilterActiveInfo& info) override;
    virtual void do_filter_toi(FilterTOIInfo& info) override;
    // Keeps no candidates: filter_active and filter_toi test every vertex.
    virtual bool do_supports_detect_skin() const noexcept override
    {
        return true;
    }
    // filter_active and filter_toi read the positions and write only this
    // filter's buffers, all on the info's stream.
    virtual bool do_runs_beside() const noexcept override { return true; }
};
}  // namespace uipc::backend::cuda
