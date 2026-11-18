/*
 * NVIDIA contributions: detection-skin support and two-phase active-set
 * filtering.
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

#include <collision_detection/trajectory_filter.h>

namespace uipc::backend::cuda
{
bool TrajectoryFilter::supports_detect_skin() const noexcept
{
    return do_supports_detect_skin();
}

void TrajectoryFilter::detect(GlobalTrajectoryFilter::DetectInfo& info)
{
    do_detect(info);
}

void TrajectoryFilter::filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info)
{
    do_filter_active(info);
}

bool TrajectoryFilter::do_launch_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info)
{
    return false;
}

void TrajectoryFilter::do_finish_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info)
{
    UIPC_ASSERT(false, "{} launched no two-phase filter_active", name());
}

bool TrajectoryFilter::launch_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info)
{
    return do_launch_filter_active(info);
}

void TrajectoryFilter::finish_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info)
{
    do_finish_filter_active(info);
}

void TrajectoryFilter::filter_toi(GlobalTrajectoryFilter::FilterTOIInfo& info)
{
    do_filter_toi(info);
}

void TrajectoryFilter::record_friction_candidates(GlobalTrajectoryFilter::RecordFrictionCandidatesInfo& info)
{
    do_record_friction_candidates(info);
}

void TrajectoryFilter::label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info)
{
    do_label_active_vertices(info);
}

void TrajectoryFilter::clear_friction_candidates()
{
    do_clear_friction_candidates();
}
}  // namespace uipc::backend::cuda
