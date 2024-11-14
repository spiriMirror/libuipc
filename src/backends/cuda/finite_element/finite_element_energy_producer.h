#pragma once
#include <sim_system.h>
#include <finite_element/finite_element_method.h>
#include <linear_system/global_linear_system.h>
#include <line_search/line_searcher.h>

namespace uipc::backend::cuda
{
class FiniteElementMethod;
class FiniteElementEnergyProducer : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    class BuildInfo
    {
      public:
    };

    class Impl
    {
      public:
        SizeT stencil_dim = 0;

        SizeT energy_offset = 0;
        SizeT energy_count  = 0;

        SizeT gradient_offset = 0;
        SizeT gradient_count  = 0;

        SizeT hessian_offset = 0;
        SizeT hessian_count  = 0;

        FiniteElementMethod* finite_element_method = nullptr;
    };

    class ReportExtentInfo
    {
      public:
        /**
         * @brief Set the number of element energy
         */
        void energy_count(SizeT count) noexcept;
        /**
         * @brief Set the stencil dimension
         *
         *  stencil_dim = N means the element contains N vertices, so
         *  the gradient has size 3 * N, and the hessian has size (3 * N) * (3 * N)
         * 
         */
        void stencil_dim(SizeT dim) noexcept;

      private:
        friend class FiniteElementEnergyProducer;
        SizeT m_energy_count = 0;
        SizeT m_stencil_dim  = 0;
    };

    class ComputeEnergyInfo
    {
      public:
        ComputeEnergyInfo(Float dt, muda::BufferView<Float> energies)
            : m_dt(dt)
            , m_energies(energies)
        {
        }
        auto energies() const noexcept { return m_energies; }
        auto dt() const noexcept { return m_dt; }

      private:
        friend class FiniteElementEnergyProducer;
        muda::BufferView<Float> m_energies;
        Float                   m_dt = 0.0;
    };

    class ComputeGradientHessianInfo
    {
      public:
        ComputeGradientHessianInfo(Float                             dt,
                                   muda::DoubletVectorView<Float, 3> gradients,
                                   muda::TripletMatrixView<Float, 3> hessians)
            : m_dt(dt)
            , m_gradients(gradients)
            , m_hessians(hessians)
        {
        }

        auto gradients() const noexcept { return m_gradients; }
        auto hessians() const noexcept { return m_hessians; }
        auto dt() const noexcept { return m_dt; }

      private:
        friend class FiniteElementEnergyProducer;
        muda::DoubletVectorView<Float, 3> m_gradients;
        muda::TripletMatrixView<Float, 3> m_hessians;
        Float                             m_dt = 0.0;
    };

  protected:
    virtual void do_build(BuildInfo& info)                  = 0;
    virtual void do_report_extent(ReportExtentInfo& info)   = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;

  private:
    friend class FEMLinearSubsystem;
    friend class FiniteElementMethod;
    friend class FEMLineSearchReporter;

    virtual void do_build() override final;
    void         collect_extent_info();
    void         compute_energy(LineSearcher::EnergyInfo& info);
    class AssemblyInfo
    {
      public:
        muda::TripletMatrixView<Float, 3> hessians;
        Float                             dt;
    };
    void assemble_gradient_hessian(AssemblyInfo& info);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
