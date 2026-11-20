/*
 * NVIDIA contributions: detection-skin, two-phase active-set filtering,
 * side-stream and read-back interfaces.
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
#include <collision_detection/global_trajectory_filter.h>

namespace uipc::backend::cuda
{
class TrajectoryFilter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  protected:
    virtual void do_detect(GlobalTrajectoryFilter::DetectInfo& info) = 0;
    virtual void do_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info) = 0;
    virtual void do_filter_toi(GlobalTrajectoryFilter::FilterTOIInfo& info) = 0;
    virtual void do_record_friction_candidates(GlobalTrajectoryFilter::RecordFrictionCandidatesInfo&) = 0;
    virtual void do_label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info) = 0;
    virtual void do_clear_friction_candidates() = 0;
    // True when do_detect widens every trajectory test that drops a pair by
    // DetectInfo::skin(), or when the filter keeps no candidates between
    // calls (see GlobalTrajectoryFilter::reuse_candidates).
    virtual bool do_supports_detect_skin() const noexcept { return false; }
    // True when the filter's two-phase filter_active and its filter_toi may
    // run on a side stream beside the other filters: the filter launches all
    // of that work on the info's stream (FilterActiveInfo::stream(),
    // FilterTOIInfo::stream()) and shares no buffer with another filter.
    virtual bool do_runs_beside() const noexcept { return false; }

    // Optional two-phase do_filter_active(), so that one host synchronization
    // serves every filter: the launch enqueues the work and the copies of its
    // host-visible counts into pinned memory without waiting and returns
    // true; the finish consumes the counts after GlobalTrajectoryFilter has
    // synchronized. A filter whose launch returns false (the default) runs
    // do_filter_active() whole.
    virtual bool do_launch_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info);
    virtual void do_finish_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info);

  private:
    friend class GlobalTrajectoryFilter;

    bool supports_detect_skin() const noexcept;
    bool runs_beside() const noexcept { return do_runs_beside(); }
    void detect(GlobalTrajectoryFilter::DetectInfo& info);
    void filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info);
    bool launch_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info);
    void finish_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info);
    void filter_toi(GlobalTrajectoryFilter::FilterTOIInfo& info);
    void record_friction_candidates(GlobalTrajectoryFilter::RecordFrictionCandidatesInfo& info);
    void label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info);
    void clear_friction_candidates();
};
}  // namespace uipc::backend::cuda
