#pragma once
#include <diff_sim/diff_parm_reporter.h>
#include <finite_element/finite_element_method.h>

namespace uipc::backend::cuda
{
class FiniteElementDiffParmReporter : public DiffParmReporter
{
  public:
    using DiffParmReporter::DiffParmReporter;

    class Impl;

    class BuildInfo
    {
      public:
    };

    using DiffParmExtentInfo = GlobalDiffSimManager::DiffParmExtentInfo;

    class DiffParmInfo
    {
      public:
        DiffParmInfo(FiniteElementDiffParmReporter::Impl* impl,
                     GlobalDiffSimManager::DiffParmInfo&  global_info,
                     Float                                dt)
            : m_impl(impl)
            , m_global_info(global_info)
            , m_dt(dt)
        {
        }

        SizeT  frame() const;
        IndexT dof_offset(SizeT frame) const;
        IndexT dof_count(SizeT frame) const;

        muda::TripletMatrixView<Float, 1> pGpP() const;
        Float                             dt() const;

      private:
        friend class Impl;
        FiniteElementDiffParmReporter::Impl* m_impl;
        GlobalDiffSimManager::DiffParmInfo&  m_global_info;
        Float                                m_dt = 0.0;
    };

    class Impl
    {
      public:
        FiniteElementMethod* fem = nullptr;
        Float                dt  = 0.0;
    };

  protected:
    virtual void do_build(BuildInfo& info)       = 0;
    virtual void do_assemble(DiffParmInfo& info) = 0;

    FiniteElementMethod::Impl& fem() const noexcept
    {
        return m_impl.fem->m_impl;
    }

  private:
    virtual void do_build(DiffParmReporter::BuildInfo& info) override final;
    virtual void do_assemble(GlobalDiffSimManager::DiffParmInfo& info) override final;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
