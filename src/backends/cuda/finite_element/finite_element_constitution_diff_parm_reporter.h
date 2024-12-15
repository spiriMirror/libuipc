#pragma once
#include <finite_element/finite_element_diff_parm_reporter.h>
#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
class FiniteElementConstitutionDiffParmReporter : public FiniteElementDiffParmReporter
{
  public:
    using FiniteElementDiffParmReporter::FiniteElementDiffParmReporter;

    U64    uid() const noexcept;
    IndexT dim() const noexcept;

    class BuildInfo
    {
      public:
    };

    using DiffParmInfo       = FiniteElementDiffParmReporter::DiffParmInfo;
    using DiffParmUpdateInfo = GlobalDiffSimManager::DiffParmUpdateInfo;

  protected:
    virtual void   do_build(BuildInfo& info)                        = 0;
    virtual U64    get_uid() const noexcept                         = 0;
    virtual IndexT get_dim() const noexcept                         = 0;
    virtual void   do_init(FiniteElementMethod::FilteredInfo& info) = 0;

    FiniteElementConstitution& constitution() noexcept;

    const FiniteElementMethod::ConstitutionInfo& constitution_info() noexcept;

  private:
    friend class FiniteElementMethod;

    virtual void do_build(FiniteElementDiffParmReporter::BuildInfo& info) override final;

    void connect();  // only be called by FiniteElementMethod

    void init();  // only be called by FiniteElementMethod

    FiniteElementConstitution* m_constitution = nullptr;
};
}  // namespace uipc::backend::cuda
