#pragma once
#include <diff_sim/diff_dof_reporter.h>
#include <finite_element/finite_element_method.h>

namespace uipc::backend::cuda
{
class FiniteElementDiffDofReporter : public DiffDofReporter
{
  public:
    using DiffDofReporter::DiffDofReporter;

    class Impl;

    class BuildInfo
    {
      public:
    };

    using DiffDofExtentInfo = GlobalDiffSimManager::DiffDofExtentInfo;

    class DiffDofInfo
    {
      public:
        DiffDofInfo(FiniteElementDiffDofReporter::Impl* impl,
                    GlobalDiffSimManager::DiffDofInfo&  global_info,
                    Float                               dt)
            : m_impl(impl)
            , m_global_info(global_info)
            , m_dt(dt)
        {
        }

        SizeT  frame() const;
        IndexT dof_offset(SizeT frame) const;
        IndexT dof_count(SizeT frame) const;

        muda::TripletMatrixView<Float, 1> H() const;
        Float                             dt() const;

      private:
        friend class Impl;
        FiniteElementDiffDofReporter::Impl* m_impl;
        GlobalDiffSimManager::DiffDofInfo&  m_global_info;
        Float                               m_dt = 0.0;
    };

    class Impl
    {
      public:
        FiniteElementMethod* fem = nullptr;
        Float                dt  = 0.0;
    };

  protected:
    virtual void do_build(BuildInfo& info)      = 0;
    virtual void do_assemble(DiffDofInfo& info) = 0;

    FiniteElementMethod::Impl& fem() const noexcept
    {
        return m_impl.fem->m_impl;
    }

  private:
    virtual void do_build(DiffDofReporter::BuildInfo& info) override final;
    virtual void do_assemble(GlobalDiffSimManager::DiffDofInfo& info) override final;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda