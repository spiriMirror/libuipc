#pragma once
#include <finite_element/finite_element_constitution_diff_parm_reporter.h>
#include <finite_element/finite_element_method.h>

namespace uipc::backend::cuda
{
class Codim1DConstitutionDiffParmReporter : public FiniteElementConstitutionDiffParmReporter
{
  public:
    using Base = FiniteElementConstitutionDiffParmReporter;
    using Base::Base;

    class DiffParmInfo
    {
      public:
        DiffParmInfo(Codim1DConstitutionDiffParmReporter* impl, Base::DiffParmInfo& diff_parm_info)
            : m_impl(impl)
            , m_diff_parm_info(diff_parm_info)
        {
        }

        cuda_tool::CBufferView<Vector3>  xs() const noexcept;
        cuda_tool::CBufferView<Vector3>  x_bars() const noexcept;
        cuda_tool::CBufferView<Float>    rest_lengths() const noexcept;
        cuda_tool::CBufferView<Vector2i> indices() const noexcept;
        cuda_tool::CBufferView<Float>    thicknesses() const noexcept;
        cuda_tool::CBufferView<IndexT>   is_fixed() const noexcept;
        const FiniteElementMethod::ConstitutionInfo& constitution_info() const noexcept;

        SizeT  frame() const;
        IndexT dof_offset(SizeT frame) const;
        IndexT dof_count(SizeT frame) const;

        cuda_tool::TripletMatrixView<Float, 1> pGpP() const;
        Float                             dt() const;

      protected:
        Codim1DConstitutionDiffParmReporter* m_impl = nullptr;
        Base::DiffParmInfo&                  m_diff_parm_info;
    };

  protected:
    virtual void do_assemble(DiffParmInfo& info) = 0;

  private:
    virtual void do_assemble(Base::DiffParmInfo& info) final override;
    IndexT       get_dim() const noexcept final override;
};
}  // namespace uipc::backend::cuda
