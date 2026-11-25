/*
 * NVIDIA contributions: interfaces for tolerance checks launched on the
 * prologue stream.
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
#include <cuda_tool/stream.h>

namespace uipc::backend::cuda
{
class NewtonToleranceChecker;

class NewtonToleranceManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class ResultInfo
    {
      public:
        bool converged() const { return m_converged; }
        void frame(SizeT frame) { m_frame = frame; }
        void newton_iter(IndexT count) { m_newton_iter = count; }

      private:
        friend class NewtonToleranceManager;
        bool   m_converged   = true;  // true if all checkers converged
        IndexT m_newton_iter = 0;
        SizeT  m_frame       = 0;  // current frame
    };

    class CheckResultInfo
    {
      public:
        // Finished iteration count
        IndexT newton_iter() const { return m_newton_iter; }
        void   converged(bool converged) { m_converged = converged; }
        // The checker's launch_check() work of this iteration has completed:
        // its results only need to be read.
        bool launched() const { return m_launched; }

      private:
        friend class NewtonToleranceManager;
        bool   m_converged   = true;  // true if this checker converged
        IndexT m_newton_iter = 0;
        bool   m_launched    = false;
    };

    class LaunchCheckInfo
    {
      public:
        // Every launch and copy of the checker's work goes to this stream.
        cudaStream_t stream() const { return m_stream; }

      private:
        friend class NewtonToleranceManager;
        cudaStream_t m_stream = cuda_tool::default_stream();
    };

    class PreNewtonInfo
    {
      public:
        SizeT frame() const { return m_frame; }

      private:
        friend class NewtonToleranceManager;
        SizeT m_frame = 0;  // current frame
    };

    class Impl
    {
      public:
        void init();

        std::string                                     report_buffer;
        SimSystemSlotCollection<NewtonToleranceChecker> tolerance_checkers;
        cudaStream_t launch_stream = cuda_tool::default_stream();
        bool         launched      = false;
    };

    virtual void do_build() override;

  private:
    friend class SimEngine;
    void init();  // only be called by SimEngine
    void pre_newton(SizeT frame);
    // Enqueue the device work of the checkers, including the copies of
    // their results to the host, on `stream` (only called by SimEngine);
    // the next check() of the Newton iteration waits for that stream and
    // only reads the results.
    void launch_check(cudaStream_t stream);
    void check(ResultInfo& info);

    friend class NewtonToleranceChecker;
    void add_checker(NewtonToleranceChecker* checker);
    Impl m_impl;
};
}  // namespace uipc::backend::cuda