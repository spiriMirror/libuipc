/*
 * NVIDIA contributions: interfaces and state for fused iteration, blockwise
 * assembly and layout generation; for the block-diagonal PCG step, assembly
 * side streams, the prelaunched and in-place FEM assembly, batched read-backs,
 * preconditioner reuse, the Newton-step slope, parallel PCG segment streams
 * and the deferred solution retrieval; for the elastic-producer prelaunch
 * right after the contact assembly launch, late Hessian blocks and the
 * preconditioner preparation from the DyTopo hook.
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
#include <functional>
#include <array>
#include <uipc/common/list.h>
#include <uipc/common/vector.h>
#include <cuda_tool/cuda_tool.h>
#include <algorithm/matrix_converter.h>
#include <linear_system/spmv.h>
#include <utils/offset_count_collection.h>
#include <energy_component_flags.h>
namespace uipc::backend::cuda
{
// Define a simple POD to avoid constructing CUDA's built-in vector type with pmr allocators in host code
struct SizeT2
{
    SizeT x;
    SizeT y;
};

class DiagLinearSubsystem;
class OffDiagLinearSubsystem;
class IterativeSolver;
class LocalPreconditioner;
class GlobalPreconditioner;

class GlobalLinearSystem : public SimSystem
{
    static constexpr SizeT DoFBlockSize = 3;

  public:
    using SimSystem::SimSystem;
    ~GlobalLinearSystem() override;
    using TripletMatrixView = cuda_tool::TripletMatrixView<Float, 3>;
    using CBCOOMatrixView   = cuda_tool::CBCOOMatrixView<Float, 3>;
    using DenseVectorView   = cuda_tool::DenseVectorView<Float>;
    using CDenseVectorView  = cuda_tool::CDenseVectorView<Float>;
    using ComponentFlags    = EnergyComponentFlags;

    class Impl;

    class InitDofExtentInfo
    {
      public:
        void extent(SizeT dof_count) noexcept { m_dof_count = dof_count; }
        // The subsystem's prelaunch can write its prefix of the global
        // system in place (PrelaunchInfo::in_place_hessians()). The first
        // diag subsystem that asks gets the first triplet range.
        void prelaunch_in_place() noexcept { m_prelaunch_in_place = true; }

      private:
        friend class Impl;
        SizeT m_dof_count          = 0;
        bool  m_prelaunch_in_place = false;
    };

    class InitDofInfo
    {
      public:
        IndexT dof_offset() const { return m_dof_offset; }
        IndexT dof_count() const { return m_dof_count; }

      private:
        friend class Impl;
        IndexT m_dof_offset = 0;
        IndexT m_dof_count  = 0;
    };

    class DiagExtentInfo
    {
      public:
        bool           gradient_only() const { return m_gradient_only; }
        ComponentFlags component_flags() const { return m_component_flags; }
        void extent(SizeT hessian_block_count, SizeT dof_count) noexcept;
        // More Hessian blocks, whose count the device determines: the
        // subsystem reports it in do_report_late_extent() and writes every
        // one of them in do_assemble_late(), both called after each
        // subsystem's do_assemble() is issued. Late blocks go behind all the
        // extent() blocks, so those are placed and written before the count
        // comes back.
        void late_extent() noexcept { m_late = true; }

      private:
        friend class Impl;
        ComponentFlags m_component_flags = ComponentFlags::All;
        SizeT          m_dof_count       = 0;
        SizeT          m_block_count     = 0;
        bool           m_gradient_only   = false;
        bool           m_late            = false;
    };

    class LateExtentInfo
    {
      public:
        void extent(SizeT hessian_block_count) noexcept
        {
            m_block_count = hessian_block_count;
        }

      private:
        friend class Impl;
        SizeT m_block_count = 0;
    };

    class PrelaunchInfo
    {
      public:
        // Side streams for the contact-independent producers launched ahead
        // of the DyTopo computation; joined by the next assembly.
        cuda_tool::SideStreams* side_streams() const { return m_side_streams; }
        // True when the call comes from DyTopo once it has launched its
        // contact assembly (and before it synchronizes with it); false in
        // prepare_prelaunch() and when there is no DyTopo computation. A
        // subsystem that must start after the contact kernels prelaunches
        // only when this is true.
        bool contact_launched() const { return m_contact_launched; }
        // True when the call comes from prelaunch_ahead(), right after the
        // latency-bound contact launch (the in-place lead only): the lead may
        // issue its elastic producers ahead of the rest of the DyTopo work,
        // held to elastic_block_cap() blocks of one SM each, so the contact
        // blocks keep the other SMs. The lead is called again once the contact
        // is launched (contact_launched()), to finish such a prelaunch or,
        // when it wrote nothing in place here, to prelaunch as usual.
        bool ahead_of_contact() const { return m_ahead_of_contact; }
        int  elastic_block_cap() const { return m_elastic_block_cap; }
        int  sm_count() const { return m_sm_count; }
        // The next (prelaunched) assembly of this subsystem writes every one
        // of the first `count` triplets of its range, unwritten slots as the
        // clear's (-1, -1, 0), so the global clear skips them.
        void fully_written_triplets(SizeT count) noexcept
        {
            m_fully_written_triplets = count;
        }

        // In-place prelaunch (InitDofExtentInfo::prelaunch_in_place()): true
        // when the subsystem may write the first `count` triplets of its
        // range and its gradient range of the global system directly, so
        // that assemble() neither gathers nor adds them. The layout is the
        // previous assembly's; the global system falls back to a full
        // assemble() when it changes.
        bool in_place_available() const noexcept;
        // Its first `count` triplets (global block indices, as
        // DiagInfo::hessians()); the capacity is ensured. Calling it commits
        // the subsystem to the in-place path for this assembly.
        TripletMatrixView in_place_hessians(SizeT count);
        // Its range of the global gradient. The global assembly does not
        // clear it after an in-place prelaunch: the subsystem clears it.
        DenseVectorView in_place_gradients();
        // The subsystem's blocked DoF offset (global block index of its
        // first DoF block) in the in-place views.
        int in_place_block_offset() const;
        // Every in-place write is enqueued on `stream` before this call:
        // the global system waits for that point before it reads A or b,
        // and before it reallocates them.
        void in_place_done(cudaStream_t stream);

      private:
        friend class Impl;
        cuda_tool::SideStreams* m_side_streams           = nullptr;
        bool                    m_contact_launched       = false;
        bool                    m_ahead_of_contact       = false;
        int                     m_elastic_block_cap      = 0;
        int                     m_sm_count               = 0;
        SizeT                   m_fully_written_triplets = 0;
        Impl*                   m_impl                   = nullptr;
        bool                    m_in_place_lead          = false;
    };

    class ComputeGradientInfo
    {
      public:
        // - Contact: only consider contact part
        // - Complement: only consider non-contact part
        void flags(ComponentFlags component) noexcept;
        void buffer_view(cuda_tool::DenseVectorView<Float> grad) noexcept;

      private:
        friend class Impl;
        cuda_tool::DenseVectorView<Float> m_gradients;
        ComponentFlags                    m_flags = ComponentFlags::All;
    };

    class DiagInfo
    {
      public:
        DiagInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        TripletMatrixView hessians() { return m_hessians; }
        DenseVectorView   gradients() { return m_gradients; }
        bool              gradient_only() const { return m_gradient_only; }
        ComponentFlags    component_flags() const { return m_component_flags; }
        // Side streams a subsystem may fork independent producers onto
        // (nullptr: assemble on the legacy default stream only). They are
        // joined into the legacy stream after every subsystem has assembled.
        cuda_tool::SideStreams* side_streams() const { return m_side_streams; }
        // True when prelaunch_assembly() already launched this subsystem's
        // contact-independent producers for this assembly.
        bool prelaunched() const { return m_prelaunched; }

      private:
        friend class Impl;
        SizeT                   m_index = ~0ull;
        TripletMatrixView       m_hessians;
        DenseVectorView         m_gradients;
        bool                    m_gradient_only   = false;
        ComponentFlags          m_component_flags = ComponentFlags::All;
        cuda_tool::SideStreams* m_side_streams    = nullptr;
        bool                    m_prelaunched     = false;
        Impl*                   m_impl            = nullptr;
    };

    class OffDiagExtentInfo
    {
      public:
        void extent(SizeT lr_hessian_block_count, SizeT rl_hassian_block_count) noexcept;

      private:
        friend class Impl;
        SizeT m_lr_block_count = 0;
        SizeT m_rl_block_count = 0;
    };

    class OffDiagInfo
    {
      public:
        OffDiagInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        TripletMatrixView lr_hessian() const { return m_lr_hessian; }
        TripletMatrixView rl_hessian() const { return m_rl_hessian; }

      private:
        friend class Impl;
        SizeT             m_index = ~0ull;
        TripletMatrixView m_lr_hessian;
        TripletMatrixView m_rl_hessian;
        Impl*             m_impl = nullptr;
    };

    class AssemblyInfo
    {
      public:
        AssemblyInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        CBCOOMatrixView A() const;

      protected:
        friend class Impl;
        Impl* m_impl = nullptr;
    };

    class GlobalPreconditionerAssemblyInfo : public AssemblyInfo
    {
      public:
        using AssemblyInfo::AssemblyInfo;
    };

    class LocalPreconditionerAssemblyInfo : public AssemblyInfo
    {
      public:
        LocalPreconditionerAssemblyInfo(Impl* impl, SizeT index, cudaStream_t stream = nullptr) noexcept
            : AssemblyInfo(impl)
            , m_index(index)
            , m_stream(stream)
        {
        }

        SizeT dof_offset() const;
        SizeT dof_count() const;
        // Stream for the assembly kernels: a side stream forked after the
        // global matrix is assembled and joined before the solve, or the
        // legacy default stream.
        cudaStream_t stream() const noexcept { return m_stream; }

      private:
        SizeT        m_index;
        cudaStream_t m_stream = nullptr;
    };

    using DiagNormInfo = LocalPreconditionerAssemblyInfo;

    class ApplyPreconditionerInfo
    {
      public:
        ApplyPreconditionerInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        DenseVectorView             z() { return m_z; }
        CDenseVectorView            r() { return m_r; }
        cuda_tool::CVarView<IndexT> converged() { return m_converged; }
        // launch stream for the apply kernels (capture stream while
        // recording a CUDA graph; legacy default stream otherwise)
        cudaStream_t stream() const noexcept { return m_stream; }

      private:
        friend class Impl;
        DenseVectorView             m_z;
        CDenseVectorView            m_r;
        cuda_tool::CVarView<IndexT> m_converged;
        cudaStream_t                m_stream = nullptr;
        Impl*                       m_impl   = nullptr;
    };

    class FusedPcgUpdateApplyDotInfo
    {
      public:
        FusedPcgUpdateApplyDotInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        DenseVectorView             x() const { return m_x; }
        CDenseVectorView            p() const { return m_p; }
        DenseVectorView             r() const { return m_r; }
        CDenseVectorView            Ap() const { return m_Ap; }
        DenseVectorView             z() const { return m_z; }
        cuda_tool::CVarView<Float>  rz() const { return m_rz; }
        cuda_tool::CVarView<Float>  pAp() const { return m_pAp; }
        cuda_tool::VarView<Float>   rz_new() const { return m_rz_new; }
        cuda_tool::CVarView<IndexT> converged() const { return m_converged; }
        cudaStream_t                stream() const noexcept { return m_stream; }

      private:
        friend class Impl;
        DenseVectorView             m_x;
        CDenseVectorView            m_p;
        DenseVectorView             m_r;
        CDenseVectorView            m_Ap;
        DenseVectorView             m_z;
        cuda_tool::CVarView<Float>  m_rz;
        cuda_tool::CVarView<Float>  m_pAp;
        cuda_tool::VarView<Float>   m_rz_new;
        cuda_tool::CVarView<IndexT> m_converged;
        cudaStream_t                m_stream = nullptr;
        Impl*                       m_impl   = nullptr;
    };

    // The block-diagonal inverse behind a local preconditioner's fused
    // update/apply/dot step, so one launch can serve several subsystems.
    class FusedPcgDiagInverseInfo
    {
      public:
        void block_12x12(cuda_tool::CBufferView<Matrix12x12> inverse) noexcept
        {
            m_block_size    = 12;
            m_inverse_12x12 = inverse;
        }
        void block_3x3(cuda_tool::CBufferView<Matrix3x3> inverse) noexcept
        {
            m_block_size  = 3;
            m_inverse_3x3 = inverse;
        }

      private:
        friend class Impl;
        int                                 m_block_size = 0;
        cuda_tool::CBufferView<Matrix12x12> m_inverse_12x12;
        cuda_tool::CBufferView<Matrix3x3>   m_inverse_3x3;
    };

    class AccuracyInfo
    {
      public:
        AccuracyInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        CDenseVectorView r() const { return m_r; }

        void satisfied(bool satisfied) { m_statisfied = satisfied; }

      private:
        friend class Impl;
        CDenseVectorView m_r;
        Impl*            m_impl       = nullptr;
        bool             m_statisfied = true;
    };

    class SolvingInfo
    {
      public:
        SolvingInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        DenseVectorView  x() { return m_x; }
        CDenseVectorView b() { return m_b; }
        void iter_count(SizeT iter_count) { m_iter_count = iter_count; }
        // The solve was only enqueued and its iteration count is read later
        // through IterativeSolver::do_pending_iter_count().
        void iter_count_pending() { m_iter_count_pending = true; }

      private:
        friend class Impl;
        DenseVectorView  m_x;
        CDenseVectorView m_b;
        SizeT            m_iter_count         = 0;
        bool             m_iter_count_pending = false;
        Impl*            m_impl               = nullptr;
    };

    class SolutionInfo
    {
      public:
        SolutionInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        CDenseVectorView solution() { return m_solution; }
        // The engine collects the vertex displacements right after this
        // solve: a subsystem may leave its retrieval to its vertex
        // reporter's report_displacements() (one kernel for both), which
        // then must run before anything reads the retrieved step.
        bool deferred() const noexcept { return m_deferred; }

      private:
        friend class Impl;
        CDenseVectorView m_solution;
        Impl*            m_impl     = nullptr;
        bool             m_deferred = false;
    };

  private:
    class LinearSubsytemInfo
    {
      public:
        bool  is_diag                  = false;
        bool  has_local_preconditioner = false;
        SizeT local_index              = ~0ull;
        SizeT index                    = ~0ull;
    };

  public:
    class Impl
    {
      public:
        void init();

        void prepare_prelaunch(SizeT beside_blocks = 0);
        void prelaunch_ahead();
        void prelaunch_assembly(bool contact_launched);
        // keep_preconditioner: keep the preconditioner of the previous
        // assembly when the DoF layout is unchanged (see
        // keeps_preconditioner()); otherwise assemble a fresh one.
        void build_linear_system(bool keep_preconditioner = false);
        bool _update_subsystem_extent();
        void _assemble_linear_system();
        void _clear_triplets();
        void _assemble_preconditioner();
        void solve_linear_system();
        void distribute_solution(bool deferred = false);
        // late blocks (DiagExtentInfo::late_extent()): their total count,
        // then their assembly behind the other blocks
        SizeT _report_late_extents();
        void  _assemble_late();
        // LocalPreconditioner::prepare_assembly(), once per assembly of the
        // preconditioners
        void prepare_preconditioners();
        bool preconditioners_prepared = false;

        Float reserve_ratio = 1.1;

        std::vector<LinearSubsytemInfo> subsystem_infos;

        OffsetCountCollection<IndexT> diag_dof_offsets_counts;
        OffsetCountCollection<IndexT> subsystem_triplet_offsets_counts;

        std::vector<SizeT2> off_diag_lr_triplet_counts;


        std::vector<int> accuracy_statisfied_flags;
        std::vector<int> no_precond_diag_subsystem_indices;

        // Segment fusion may update x/r once per local preconditioner only
        // when each subsystem has one owner. Fixed after initialization.
        bool local_preconditioner_owners_unique = true;

        // Containers
        SimSystemSlotCollection<DiagLinearSubsystem>    diag_subsystems;
        SimSystemSlotCollection<OffDiagLinearSubsystem> off_diag_subsystems;
        SimSystemSlotCollection<LocalPreconditioner>    local_preconditioners;

        SimSystemSlot<IterativeSolver>      iterative_solver;
        SimSystemSlot<GlobalPreconditioner> global_preconditioner;

        // Linear System
        cuda_tool::LinearSystemContext           ctx;
        cuda_tool::DeviceDenseVector<Float>      x;
        cuda_tool::DeviceDenseVector<Float>      b;
        cuda_tool::DeviceTripletMatrix<Float, 3> triplet_A;
        cuda_tool::DeviceBCOOMatrix<Float, 3>    bcoo_A;
        cuda_tool::DeviceDenseMatrix<Float>      debug_A;  // dense A for debug
        // device-side copy of bcoo_A's triplet count (for graph-stable SpMV)
        cuda_tool::DeviceVar<IndexT> triplet_count_dev;

        Spmv                      spmver;
        MatrixConverter<Float, 3> converter;

        // Side streams for independent assembly producers (see
        // DiagInfo::side_streams) and local preconditioner assembly.
        cuda_tool::SideStreams assembly_streams;
        bool                   assembly_prelaunched = false;
        bool                   prelaunch_prepared   = false;
        // the in-place lead prelaunched ahead of the contact assembly
        bool lead_ahead = false;
        // prepare_prelaunch()'s beside_blocks, for prelaunch_ahead()
        SizeT ahead_blocks = 0;
        int   sm_count     = 0;
        // per diag subsystem: prefix of its triplet range its prelaunched
        // assembly writes in full (PrelaunchInfo::fully_written_triplets)
        std::vector<SizeT> diag_fully_written_triplets;

        // In-place prelaunch (PrelaunchInfo::in_place_hessians()): the diag
        // subsystem (local index) whose triplet range goes first, so its
        // prefix sits at a position that does not depend on the other
        // subsystems' (contact-dependent) counts; -1 without one.
        IndexT in_place_lead = -1;
        // A previous assembly fixed the lead's DoF range and block offset.
        bool in_place_ready = false;
        // This assembly's prelaunch wrote in place (in_place_event marks it).
        bool        in_place_written = false;
        bool        in_place_waited  = false;
        cudaEvent_t in_place_event   = nullptr;
        // The DoF layout changed after an in-place prelaunch: the lead
        // assembles in full instead (DiagInfo::prelaunched() is false).
        bool in_place_discarded = false;
        // Subsystem triplet offsets (the lead's range first, then the others
        // in subsystem order) and the subsystems in triplet order.
        std::vector<IndexT> triplet_offsets;
        std::vector<SizeT>  triplet_order;
        // The legacy stream waits for the in-place writes (once per assembly).
        void wait_in_place();
        // Per diag subsystem (local index): this assembly has late blocks
        // (DiagExtentInfo::late_extent()), and their reported count. They
        // follow the triplet_order ranges, in subsystem order.
        std::vector<char>  diag_late;
        std::vector<SizeT> diag_late_counts;
        bool               late_pending  = false;
        bool               late_reported = false;

        // One side stream per local preconditioner but the last: the fused
        // PCG update/apply/dot segments are independent (disjoint DoF ranges,
        // atomic r^T z), so a captured iteration runs them as parallel branches.
        cuda_tool::SideStreams pcg_segment_streams;

        bool  initialized           = false;
        bool  empty_system          = true;
        SizeT last_solve_iterations = 0;
        // The solver has not read the last iteration count back yet.
        bool last_solve_pending = false;
        // Changes whenever the per-subsystem DoF partition changes. CUDA
        // graphs capture subview offsets/counts and local-preconditioner
        // buffer pointers, so total DoF and global matrix pointers alone are
        // not a sufficient graph validity key.
        SizeT dof_layout_generation = 0;

        // Preconditioner reuse across Newton iterations: opt-in through
        // linear_system/keep_preconditioner (default 0), standard IPC
        // pipeline only, fixed at build; see keeps_preconditioner().
        bool preconditioner_reuse = false;
        // DoF layout generation of the last preconditioner assembly; the
        // initial value matches no layout, so the first solve assembles.
        SizeT preconditioner_layout_generation = ~SizeT{0};

        void apply_preconditioner(cuda_tool::DenseVectorView<Float>  z,
                                  cuda_tool::CDenseVectorView<Float> r,
                                  cuda_tool::CVarView<IndexT>        converged,
                                  cudaStream_t stream = nullptr);
        bool fused_pcg_update_apply_dot(cuda_tool::DenseVectorView<Float>  x,
                                        cuda_tool::CDenseVectorView<Float> p,
                                        cuda_tool::DenseVectorView<Float>  r,
                                        cuda_tool::CDenseVectorView<Float> Ap,
                                        cuda_tool::DenseVectorView<Float>  z,
                                        cuda_tool::CVarView<Float>         rz,
                                        cuda_tool::CVarView<Float>         pAp,
                                        cuda_tool::VarView<Float>   rz_new,
                                        cuda_tool::CVarView<IndexT> converged,
                                        cudaStream_t                stream);

        void spmv(Float                              a,
                  cuda_tool::CDenseVectorView<Float> x,
                  Float                              b,
                  cuda_tool::DenseVectorView<Float>  y);
        void spmv_dot(cuda_tool::CDenseVectorView<Float> x,
                      cuda_tool::DenseVectorView<Float>  y,
                      cuda_tool::VarView<Float>          d_dot,
                      cudaStream_t                       stream = nullptr);
        void spmv_dot_pipelined(cuda_tool::CDenseVectorView<Float> x,
                                cuda_tool::DenseVectorView<Float>  y,
                                cuda_tool::VarView<Float>          d_dot,
                                cuda_tool::DenseVectorView<Float>  next_y,
                                cuda_tool::VarView<Float>          next_dot,
                                cuda_tool::CVarView<IndexT>        converged,
                                cudaStream_t                       stream);

        // data pointers of the assembled matrix buffers (row/col/value),
        // for the FusedPCG CUDA-graph validity key: a reassembly that
        // reallocates any of them invalidates the captured graph
        std::array<const void*, 3> matrix_data_ptrs() const
        {
            auto v = bcoo_A.cview();
            return {v.row_indices().data(), v.col_indices().data(), v.values().data()};
        }


        bool accuracy_statisfied(cuda_tool::DenseVectorView<Float> r);
        void compute_gradient(ComputeGradientInfo& info);

        Float diag_norm();
        Float mass_norm();

        bool        need_debug_dump = false;
        std::string debug_dump_path;
    };

    SizeT dof_count() const;
    // Iteration count of the last solve. A solver may enqueue the solve
    // without reading the count back; then this waits for that solve, so
    // callers should ask after their next host synchronization.
    SizeT last_solve_iterations();

    // Newton iterations that keep the preconditioner assembled for the
    // previous iteration when linear_system/keep_preconditioner is 1 (default
    // 0): every odd iteration from the third on. Iterations 0 and 1 (the
    // large first steps of a frame) and every even iteration assemble a fresh
    // one, so a kept preconditioner is one Newton step old. PCG keeps its
    // tol_rate, but measures it in the r^T z norm of the kept preconditioner,
    // which can stop the solve early; see do_build().
    static constexpr bool keeps_preconditioner(SizeT newton_iter) noexcept
    {
        return newton_iter >= 3 && newton_iter % 2 == 1;
    }
    void compute_gradient(ComputeGradientInfo& info);

    cuda_tool::LinearSystemContext& ctx() noexcept { return m_impl.ctx; }

  protected:
    void do_build() override;

  private:
    friend class SimEngine;
    friend class IterativeSolver;
    friend class DiagLinearSubsystem;
    friend class OffDiagLinearSubsystem;
    friend class LocalPreconditioner;
    friend class GlobalPreconditioner;
    friend class GlobalDiffSimManager;
    friend class CurrentFrameDiffDofReporter;
    friend class DuplicateLocalPreconditionerTestAccess;
    friend class PreconditionerReuseTestAccess;
    friend class InPlacePrelaunchTestAccess;

    void add_subsystem(DiagLinearSubsystem* subsystem);
    void add_subsystem(OffDiagLinearSubsystem* subsystem);
    void add_solver(IterativeSolver* solver);
    void add_preconditioner(LocalPreconditioner* preconditioner);
    void add_preconditioner(GlobalPreconditioner* preconditioner);

    // only be called by SimEngine::do_init();
    void init();

    // only be called by SimEngine::do_advance(), before the DyTopo effect of
    // the Newton iteration whose system solve() then assembles: sizes the
    // prelaunched producers' buffers and forks the side streams from the
    // legacy stream (prelaunch_assembly() does it when this was not called).
    // beside_blocks: one-warp blocks of the latency-bound DyTopo assembly
    // kernels to come (GlobalDyTopoEffectManager::assembly_blocks()). When
    // nonzero, the in-place lead may prelaunch in prelaunch_ahead(), right
    // after them, its elastic producers held to the SMs those blocks leave
    // (at least half).
    void prepare_prelaunch(SizeT beside_blocks = 0);

    // only be called by SimEngine::do_advance(), from the DyTopo hook once
    // the latency-bound assembly kernels are launched (before the others)
    void prelaunch_ahead();

    // only be called by SimEngine::do_advance(), once per Newton iteration
    // after prepare_prelaunch() and before solve(): launches the
    // contact-independent producers, with contact_launched = true from the
    // DyTopo hook once the contact assembly is enqueued (false without DyTopo)
    void prelaunch_assembly(bool contact_launched);

    // only be called by SimEngine::do_advance(); defer_retrieve: the
    // displacements are collected right after (SolutionInfo::deferred())
    void solve(bool defer_retrieve = false);

    // only be called by SimEngine::do_advance()
    Float diag_norm();
    Float mass_norm();

    // only be called by SimEngine::do_advance(): the energy slope g.dx along
    // the last solved Newton step dx = -x, where A x = b and b is the gradient
    Float newton_step_slope();

    Impl m_impl;

    // local debug dump functions
    void _dump_A_b();
    void _dump_x();
};
}  // namespace uipc::backend::cuda
