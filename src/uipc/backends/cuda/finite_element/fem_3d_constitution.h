#pragma once
#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
class FEM3DConstitution : public FiniteElementConstitution
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
    virtual void do_retrieve(FiniteElementMethod::FEM3DFilteredInfo& info) = 0;

  private:
    friend class FiniteElementMethod;

    void retrieve(FiniteElementMethod::FEM3DFilteredInfo& info);
    virtual void do_build(FiniteElementConstitution::BuildInfo& info) override final;
};
}  // namespace uipc::backend::cuda
