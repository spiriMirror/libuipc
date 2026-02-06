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

    class BaseInfo
    {
      public:
        BaseInfo(FEM3DConstitution* impl, SizeT index_in_dim, Float dt)
            : m_impl(impl)
            , m_index_in_dim(index_in_dim)
            , m_dt(dt)
        {
        }

        muda::CBufferView<Vector3>   xs() const noexcept;
        muda::CBufferView<Vector3>   x_bars() const noexcept;
        muda::CBufferView<Matrix3x3> Dm_invs() const noexcept;
        muda::CBufferView<Float>     rest_volumes() const noexcept;
        muda::CBufferView<Vector4i>  indices() const noexcept;
        const FiniteElementMethod::ConstitutionInfo& constitution_info() const noexcept;
        auto dt() const noexcept { return m_dt; }

      protected:
        SizeT              m_index_in_dim = ~0ull;
        FEM3DConstitution* m_impl         = nullptr;
        Float              m_dt           = 0.0;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        ComputeEnergyInfo(FEM3DConstitution*      impl,
                          SizeT                   index_in_dim,
                          Float                   dt,
                          muda::BufferView<Float> energies)
            : BaseInfo(impl, index_in_dim, dt)
            , m_energies(energies)
        {
        }

        auto energies() const noexcept { return m_energies; }

      private:
        muda::BufferView<Float> m_energies;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        ComputeGradientHessianInfo(FEM3DConstitution* impl,
                                   SizeT              index_in_dim,
                                   bool               gradient_only,
                                   Float              dt,
                                   muda::DoubletVectorView<Float, 3> gradients,
                                   muda::TripletMatrixView<Float, 3> hessians)
            : BaseInfo(impl, index_in_dim, dt)
            , m_gradients(gradients)
            , m_hessians(hessians)
            , m_gradient_only(gradient_only)
        {
        }

        auto gradients() const noexcept { return m_gradients; }
        auto hessians() const noexcept { return m_hessians; }
        auto gradient_only() const noexcept { return m_gradient_only; }

      private:
        muda::DoubletVectorView<Float, 3> m_gradients;
        muda::TripletMatrixView<Float, 3> m_hessians;
        bool                              m_gradient_only = false;
    };

  protected:
    virtual void do_build(BuildInfo& info)                  = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;

  private:
    friend class FiniteElementMethod;
    virtual void do_build(FiniteElementConstitution::BuildInfo& info) override final;
    virtual void do_compute_energy(FiniteElementConstitution::ComputeEnergyInfo& info) override final;
    virtual void do_compute_gradient_hessian(
        FiniteElementConstitution::ComputeGradientHessianInfo& info) override final;
    virtual IndexT get_dim() const noexcept override final;
};
}  // namespace uipc::backend::cuda
