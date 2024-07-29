#pragma once
#include <sim_system.h>
#include <finite_element/finite_element_method.h>

namespace uipc::backend::cuda
{
class FiniteElementConstitution : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    U64    constitution_uid() const;
    IndexT dimension() const;

    class BuildInfo
    {
      public:
    };

  protected:
    virtual U64    get_constitution_uid() const = 0;
    virtual IndexT get_dimension() const        = 0;
    virtual void   do_build(BuildInfo& info)    = 0;

  private:
    friend class FiniteElementMethod;
    friend class FEMLineSearchReporter;

    virtual void do_build() override final;

    SizeT m_index        = ~0ull;
    SizeT m_index_in_dim = ~0ull;
};
}  // namespace uipc::backend::cuda
