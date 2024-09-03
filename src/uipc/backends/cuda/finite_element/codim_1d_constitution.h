#pragma once
#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
class Codim1DConstitution : public FiniteElementConstitution
{
  public:
    using FiniteElementConstitution::FiniteElementConstitution;

    class BuildInfo
    {
      public:
    };

    class BaseInfo
    {
      public:
        BaseInfo(FiniteElementMethod::Impl* impl, SizeT index_in_dim, Float dt)
            : m_fem(impl)
            , m_index_in_dim(index_in_dim)
            , m_dt(dt)
        {
        }

        muda::CBufferView<Vector3>  xs() const noexcept;
        muda::CBufferView<Vector3>  x_bars() const noexcept;
        muda::CBufferView<Float>    rest_lengths() const noexcept;
        muda::CBufferView<Vector2i> indices() const noexcept;
        muda::CBufferView<Float>    thicknesses() const noexcept;
        const FiniteElementMethod::ConstitutionInfo& constitution_info() const noexcept;
        Float dt() const noexcept;

      protected:
        SizeT                      m_index_in_dim = ~0ull;
        FiniteElementMethod::Impl* m_fem          = nullptr;
        Float                      m_dt           = 0.0;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;

        muda::BufferView<Float> element_energies() const noexcept;

      private:
        friend class Codim1DConstitution;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;

        muda::BufferView<Vector6>   gradient() const noexcept;
        muda::BufferView<Matrix6x6> hessian() const noexcept;

      private:
        friend class Codim1DConstitution;
    };


  protected:
    virtual void do_retrieve(FiniteElementMethod::Codim1DFilteredInfo& info) = 0;
    virtual void do_build(BuildInfo& info)                  = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;

  private:
    friend class FiniteElementMethod;
    void retrieve(FiniteElementMethod::Codim1DFilteredInfo& info);
    virtual void do_build(FiniteElementConstitution::BuildInfo& info) override final;
    void do_compute_energy(FiniteElementMethod::ComputeEnergyInfo& info) override;
    void do_compute_gradient_hessian(FiniteElementMethod::ComputeGradientHessianInfo& info) override;
    virtual IndexT get_dimension() const override final;
};
}  // namespace uipc::backend::cuda
