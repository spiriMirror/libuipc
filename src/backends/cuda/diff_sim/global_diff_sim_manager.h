#pragma once
#include <sim_system.h>
#include <muda/ext/linear_system.h>
#include <utils/offset_count_collection.h>
#include <algorithm/matrix_converter.h>
#include <uipc/diff_sim/sparse_coo_view.h>

namespace uipc::backend::cuda
{
class DiffDofReporter;
class DiffParmReporter;
class GlobalLinearSystem;

class GlobalDiffSimManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class SparseCOO
    {
      public:
        vector<IndexT> row_indices;
        vector<IndexT> col_indices;
        vector<Float>  values;
        Vector2i       shape;

        diff_sim::SparseCOOView view() const;
    };

    class Impl
    {
      public:
        muda::LinearSystemContext& ctx();

        void init(WorldVisitor& world);
        void update();
        void assemble();
        void write_scene(WorldVisitor& world);

        GlobalLinearSystem* global_linear_system = nullptr;
        SimEngine*          sim_engine           = nullptr;

        SimSystemSlotCollection<DiffDofReporter>  diff_dof_reporters;
        SimSystemSlotCollection<DiffParmReporter> diff_parm_reporters;

        OffsetCountCollection<IndexT> diff_dof_triplet_offset_count;
        OffsetCountCollection<IndexT> diff_parm_triplet_offset_count;

        vector<IndexT> dof_offsets;
        vector<IndexT> dof_counts;

        SizeT total_parm_count = 0;

        muda::DeviceBuffer<Float> parameters;

        // NOTE:
        // local_triplet_pGpP only consider the triplet at current frame.
        // So the triplet count = current_frame_triplet_count
        // The shape of the matrix = (total_frame_dof_count, total_parm_count)
        //tex:
        //$$
        //T = \begin{bmatrix}
        // 0   \\
        // ... \\
        // 0   \\
        // T^{[i]} \\
        //\end{bmatrix}
        //$$
        muda::DeviceTripletMatrix<Float, 1> local_triplet_pGpP;
        //tex:
        //$$
        //T = \begin{bmatrix}
        // T^{[1]}   \\
        // ... \\
        // T^{[i]} \\
        //\end{bmatrix}
        //$$
        muda::DeviceTripletMatrix<Float, 1> total_triplet_pGpP;
        muda::DeviceCOOMatrix<Float>        total_coo_pGpP;

        // NOTE:
        // local_triplet_H only consider the triplet at current frame.
        // So the triplet count = current_frame_triplet_count
        // The shape of the matrix = (total_frame_dof_count, total_frame_dof_count)
        //tex:
        //$$
        //T = \begin{bmatrix}
        // 0   \\
        // ... \\
        // 0   \\
        // H^{[i]} \\
        //\end{bmatrix}
        //$$
        muda::DeviceTripletMatrix<Float, 1> local_triplet_H;
        //tex:
        //$$
        //T = \begin{bmatrix}
        // H^{[1]}   \\
        // ... \\
        // H^{[i]} \\
        //\end{bmatrix}
        //$$
        muda::DeviceTripletMatrix<Float, 1> total_triplet_H;
        muda::DeviceCOOMatrix<Float>        total_coo_H;

        SizeT current_frame_dof_count = 0;
        SizeT total_frame_dof_count   = 0;

        SparseCOO host_coo_pGpP;
        SparseCOO host_coo_H;
    };

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl, SizeT index)
            : m_impl(impl)
            , m_index(index)
        {
        }

        SizeT  frame() const;
        IndexT dof_offset(SizeT frame) const;
        IndexT dof_count(SizeT frame) const;

      protected:
        friend class Impl;
        Impl* m_impl  = nullptr;
        SizeT m_index = ~0ull;
    };

    class DiffParmExtentInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
        void triplet_count(SizeT N) { m_triplet_count = N; }

      private:
        friend class Impl;
        SizeT m_triplet_count = 0;
    };


    class DiffParmInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
        muda::TripletMatrixView<Float, 1> pGpP() const;
    };

    class DiffDofExtentInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
        void triplet_count(SizeT N) { m_triplet_count = N; }

      private:
        friend class Impl;
        SizeT m_triplet_count = 0;
    };


    class DiffDofInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;

        friend class Impl;
        muda::TripletMatrixView<Float, 1> H() const;
    };

    class DiffParmUpdateInfo
    {
      public:
        DiffParmUpdateInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        muda::CBufferView<Float> parameters() const noexcept;

      private:
        friend class Impl;
        Impl* m_impl = nullptr;
    };

  private:
    friend class SimEngine;
    void init();      // only be called by SimEngine
    void assemble();  // only be called by SimEngine
    void update();    // only be called by SimEngine

    virtual void do_build() override;


    friend class DiffDofReporter;
    friend class DiffParmReporter;

    void add_reporter(DiffDofReporter* subsystem);  // only be called by DiffDofReporter
    void add_reporter(DiffParmReporter* subsystem);  // only be called by DiffParmReporter

    Impl m_impl;
};
}  // namespace uipc::backend::cuda