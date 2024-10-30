#pragma once
#include <sim_system.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_energy_producer.h>

namespace uipc::backend::cuda
{
class FiniteElementExtraConstitution : public FiniteElementEnergyProducer
{
  public:
    using FiniteElementEnergyProducer::FiniteElementEnergyProducer;

    class Impl
    {
      public:
        void init(U64 uid, backend::WorldVisitor& world);

        FiniteElementMethod*                 finite_element_method = nullptr;
        vector<FiniteElementMethod::GeoInfo> geo_infos;
        FiniteElementMethod::Impl&           fem() noexcept
        {
            return finite_element_method->m_impl;
        }
    };

    class BuildInfo
    {
      public:
    };

    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        span<const FiniteElementMethod::GeoInfo> geo_infos() const noexcept;

        template <typename ForEach, typename ViewGetter>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ViewGetter&&                    view_getter,
                      ForEach&&                       for_each_action);

        template <typename ForEach>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots, ForEach&& for_each_action);

        span<const Vector3> positions() noexcept;
        span<const Vector3> rest_positions() noexcept;
        span<const Float>   thicknesses() noexcept;

      private:
        Impl* m_impl = nullptr;
    };

    class BaseInfo
    {
      public:
        BaseInfo(FiniteElementMethod::Impl* impl, Float dt)
            : m_impl(impl)
            , m_dt(dt)
        {
        }

        Float dt() const noexcept;

        muda::CBufferView<Vector3> xs() const noexcept;
        muda::CBufferView<Vector3> x_bars() const noexcept;
        muda::CBufferView<IndexT>  is_fixed() const noexcept;
        muda::CBufferView<Float>   thicknesses() const noexcept;

      protected:
        FiniteElementMethod::Impl* m_impl = nullptr;
        Float                      m_dt;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        ComputeEnergyInfo(FiniteElementMethod::Impl* impl, Float dt, muda::BufferView<Float> energies)
            : BaseInfo(impl, dt)
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
        ComputeGradientHessianInfo(FiniteElementMethod::Impl*        impl,
                                   Float                             dt,
                                   muda::DoubletVectorView<Float, 3> gradients,
                                   muda::TripletMatrixView<Float, 3> hessians)
            : BaseInfo(impl, dt)
            , m_gradients(gradients)
            , m_hessians(hessians)
        {
        }

        auto gradients() const noexcept { return m_gradients; }
        auto hessians() const noexcept { return m_hessians; }

      private:
        muda::DoubletVectorView<Float, 3> m_gradients;
        muda::TripletMatrixView<Float, 3> m_hessians;
    };

    U64 uid() const noexcept;

  protected:
    virtual void do_build(BuildInfo& info)                  = 0;
    virtual U64  get_uid() const noexcept                   = 0;
    virtual void do_init(FilteredInfo& info)                = 0;
    virtual void do_report_extent(ReportExtentInfo& info)   = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;

    friend class FiniteElementExtraConstitutionDiffParmReporter;
    span<const FiniteElementMethod::GeoInfo> geo_infos() const noexcept;

  private:
    friend class FiniteElementMethod;
    void init();  // only be called by FiniteElementMethod
    virtual void do_build(FiniteElementEnergyProducer::BuildInfo& info) override final;
    friend class FEMLineSearchReporter;
    virtual void do_compute_energy(FiniteElementEnergyProducer::ComputeEnergyInfo& info) override final;
    friend class FEMGradientHessianComputer;
    virtual void do_compute_gradient_hessian(
        FiniteElementEnergyProducer::ComputeGradientHessianInfo& info) override final;
    Impl m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/finite_element_extra_constitution.inl"