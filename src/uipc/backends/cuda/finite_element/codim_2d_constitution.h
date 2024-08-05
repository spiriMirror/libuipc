#pragma once
#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
class Codim2DConstitution : public FiniteElementConstitution
{
  public:
    using FiniteElementConstitution::FiniteElementConstitution;

    class BuildInfo
    {
      public:
    };

  protected:
    virtual void   do_build(BuildInfo& info) = 0;
    virtual IndexT get_dimension() const override;

  private:
    friend class FiniteElementMethod;
    void retrieve(FiniteElementMethod::Codim2DFilteredInfo& info);
    virtual void do_build(FiniteElementConstitution::BuildInfo& info) override final;
};
}  // namespace uipc::backend::cuda
