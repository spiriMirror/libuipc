#pragma once
#include <sim_system.h>
#include <muda/ext/linear_system.h>

namespace uipc::backend::cuda
{
class DiffDofRerporter;
class DiffParmRerporter;

class GlobalDiffSimManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl
    {
      public:
        void init();

        SimSystemSlotCollection<DiffDofRerporter>  diff_dof_reporters;
        SimSystemSlotCollection<DiffParmRerporter> diff_parm_reporters;

        muda::DeviceTripletMatrix<Float, 1> triplet_H;
        muda::DeviceTripletMatrix<Float, 1> triplet_pGpP;

        muda::DeviceCOOMatrix<Float> coo_H;
        muda::DeviceCOOMatrix<Float> coo_pGpP;
    };

    class DiffPamExtentInfo
    {
      public:
        void triplet_count();

      private:
        SizeT m_triplet_count = 0;
    };

    class DiffDofExtentInfo
    {
      public:
        void triplet_count();

      private:
        SizeT m_triplet_count = 0;
    };

    class DiffParmInfo
    {
      public:
        muda::COOMatrixView<Float> H() const;
    };

    class DiffDofInfo
    {
      public:
        muda::COOMatrixView<Float> H() const;
    };

  private:
    friend class SimEngine;
    void init();  // only be called by SimEngine

    virtual void do_build() override;
    void         write_scene();
    friend class DiffSimSubsystem;

    void add_subsystem(DiffDofRerporter* subsystem);
    void add_subsystem(DiffParmRerporter* subsystem);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda