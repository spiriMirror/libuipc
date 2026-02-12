#pragma once
#include <sim_system.h>
#include <muda/ext/linear_system/device_doublet_vector.h>
#include <muda/ext/linear_system/device_triplet_matrix.h>
#include <finite_element/finite_element_method.h>
#include <utils/offset_count_collection.h>
#include <finite_element/fem_linear_subsystem.h>
#include <finite_element/fem_line_search_reporter.h>

namespace uipc::backend::cuda
{
class FiniteElementMethod;

class FiniteElementElastics final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class ReportExtentInfo
    {
      public:
        void energy_count(SizeT count) noexcept { m_energy_count = count; }
        void gradient_count(SizeT count) noexcept { m_gradient_count = count; }
        void hessian_count(SizeT count) noexcept { m_hessian_count = count; }
        bool gradient_only() const noexcept
        {
            m_gradient_only_checked = true;
            return m_gradient_only;
        }
        void check(std::string_view name) const;

      private:
        friend class FiniteElementElastics;
        friend class FiniteElementConstitution;
        friend class FiniteElementExtraConstitution;

        SizeT m_energy_count   = 0;
        SizeT m_gradient_count = 0;
        SizeT m_hessian_count  = 0;
        bool  m_gradient_only  = false;
        mutable bool m_gradient_only_checked = false;
    };

    class ComputeEnergyInfo
    {
      public:
        ComputeEnergyInfo(Impl* impl, SizeT index, Float dt, muda::BufferView<Float> energies)
            : m_impl(impl)
            , m_index(index)
            , m_dt(dt)
            , m_energies(energies)
        {
        }

        auto energies() const noexcept
        {
            auto [offset, count] = m_impl->constitution_energy_offsets_counts[m_index];
            return m_energies.subview(offset, count);
        }
        auto dt() const noexcept { return m_dt; }

      private:
        Impl*                   m_impl  = nullptr;
        SizeT                   m_index = 0;
        Float                   m_dt    = 0.0;
        muda::BufferView<Float> m_energies;
    };

    class ComputeGradientHessianInfo
    {
      public:
        ComputeGradientHessianInfo(Impl* impl,
                                   SizeT index,
                                   bool  gradient_only,
                                   Float dt,
                                   muda::DoubletVectorView<Float, 3> gradients,
                                   muda::TripletMatrixView<Float, 3> hessians)
            : m_impl(impl)
            , m_index(index)
            , m_gradient_only(gradient_only)
            , m_dt(dt)
            , m_gradients(gradients)
            , m_hessians(hessians)
        {
        }

        auto gradient_only() const noexcept { return m_gradient_only; }
        muda::DoubletVectorView<Float, 3> gradients() const noexcept;
        muda::TripletMatrixView<Float, 3> hessians() const noexcept;

        auto dt() const noexcept { return m_dt; }

      private:
        Impl*                             m_impl          = nullptr;
        SizeT                             m_index         = 0;
        bool                              m_gradient_only = false;
        Float                             m_dt            = 0.0;
        muda::DoubletVectorView<Float, 3> m_gradients;
        muda::TripletMatrixView<Float, 3> m_hessians;
    };

    class Impl
    {
      public:
        SimSystemSlot<FiniteElementMethod> finite_element_method;
        FiniteElementMethod::Impl&         fem()
        {
            return finite_element_method->m_impl;
        }

        OffsetCountCollection<IndexT> constitution_energy_offsets_counts;
        OffsetCountCollection<IndexT> constitution_gradient_offsets_counts;
        OffsetCountCollection<IndexT> constitution_hessian_offsets_counts;

        void assemble(FEMLinearSubsystem::AssembleInfo& info);
        void compute_energy(FEMLineSearchReporter::ComputeEnergyInfo& info);
    };

  private:
    friend class FiniteElementElasticsLinearSubsystemReporter;
    friend class FiniteElementElasticsLineSearchSubreporter;

    virtual void do_build() override;

    friend class FiniteElementMethod;
    void init();  // only be called in FiniteElementMethod

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
