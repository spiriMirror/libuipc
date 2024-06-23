#pragma once
#include <sim_system.h>
#include <functional>
#include <uipc/common/list.h>
#include <uipc/common/vector.h>
#include <muda/ext/linear_system.h>
namespace uipc::backend::cuda
{
class DiagLinearSubsystem;
class OffDiagLinearSubsystem;

class GlobalLinearSystem : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    using TripletMatrixView = muda::TripletMatrixView<Float, 3>;
    using DenseVectorView   = muda::DenseVectorView<Float>;
    using CDenseVectorView  = muda::CDenseVectorView<Float>;

    void add_subsystem(DiagLinearSubsystem* subsystem);
    void add_subsystem(OffDiagLinearSubsystem* subsystem);

    class Impl;

    enum class HessianStorageType
    {
        Full      = 0,
        Symmetric = 1
    };

    class DiagExtentInfo
    {
      public:
        void extent(SizeT              hessian_block_count,
                    SizeT              dof_count,
                    HessianStorageType storage = HessianStorageType::Full)
        {
            m_block_count  = hessian_block_count;
            m_dof_count    = dof_count;
            m_storage_type = storage;
        }

      private:
        friend class Impl;
        SizeT              m_dof_count;
        SizeT              m_block_count;
        HessianStorageType m_storage_type;
    };

    class DiagInfo
    {
      public:
        DiagInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        TripletMatrixView hessian() { return m_hessian; }
        DenseVectorView   gradient() { return m_gradient; }

      private:
        SizeT             m_index;
        TripletMatrixView m_hessian;
        DenseVectorView   m_gradient;
        Impl*             m_impl;
    };

    class SolutionInfo
    {
      public:
        SolutionInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        CDenseVectorView solution() { return m_solution; }

      private:
        SizeT            m_index;
        CDenseVectorView m_solution;
        Impl*            m_impl;
    };

    class OffDiagExtentInfo
    {
      public:
        void extent(SizeT              lr_hessian_block_count,
                    SizeT              rl_hassian_block_count,
                    HessianStorageType storage = HessianStorageType::Full)
        {
            m_lr_block_count = lr_hessian_block_count;
            m_rl_block_count = rl_hassian_block_count;
            m_storage_type   = storage;
        }

      private:
        friend class Impl;
        SizeT              m_lr_block_count;
        SizeT              m_rl_block_count;
        HessianStorageType m_storage_type;
    };

    class OffDiagInfo
    {
      public:
        OffDiagInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        TripletMatrixView lr_hessian() { return m_lr_hessian; }
        TripletMatrixView rl_hessian() { return m_rl_hessian; }

      private:
        SizeT             m_index;
        TripletMatrixView m_lr_hessian;
        TripletMatrixView m_rl_hessian;
        Impl*             m_impl;
    };

  private:
    class LinearSubsytemInfo
    {
      public:
        bool  is_diag     = false;
        SizeT local_index = ~0ull;
        SizeT index       = ~0ull;
    };

  public:
    class Impl
    {
      public:
        void init();
        void add_subsystem(DiagLinearSubsystem* subsystem);
        void add_subsystem(OffDiagLinearSubsystem* subsystem);


        void build_linear_system();
        void _update_subsystem_extent();
        void solve_linear_system();
        void distribute_solution();

        list<DiagLinearSubsystem*>    diag_subsystem_buffer;
        list<OffDiagLinearSubsystem*> off_diag_subsystem_buffer;

        vector<DiagLinearSubsystem*>    diag_subsystems;
        vector<OffDiagLinearSubsystem*> off_diag_subsystems;
        vector<LinearSubsytemInfo>      subsystem_infos;

        muda::LinearSystemContext           ctx;
        muda::DeviceDenseVector<Float>      x;
        muda::DeviceDenseVector<Float>      b;
        muda::DeviceTripletMatrix<Float, 3> triplet_A;
        muda::DeviceBCOOMatrix<Float, 3>    bcoo_A;

        muda::DeviceBSRMatrix<Float, 3> bsr_A;
        muda::DeviceCSRMatrix<Float>    csr_A;

        vector<SizeT> subsystem_triplet_offsets;
        vector<SizeT> subsystem_triplet_counts;

        vector<SizeT> diag_dof_offsets;
        vector<SizeT> diag_dof_counts;

        //Spmv      m_spmv;
        //Converter m_converter;
    };

  protected:
    void do_build() override;

  private:
    friend class SimEngine;
    void solve();
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
