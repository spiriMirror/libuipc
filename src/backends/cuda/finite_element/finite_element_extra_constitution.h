#pragma once
#include <sim_system.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_elastics.h>
#include <muda/buffer.h>
#include <muda/ext/linear_system/device_doublet_vector.h>
#include <muda/ext/linear_system/device_triplet_matrix.h>

namespace uipc::backend::cuda
{
class FiniteElementExtraConstitution : public SimSystem
{
  public:
    using SimSystem::SimSystem;

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

    using ReportExtentInfo  = FiniteElementElastics::ReportExtentInfo;
    using ComputeEnergyInfo = FiniteElementElastics::ComputeEnergyInfo;
    using ComputeGradientHessianInfo = FiniteElementElastics::ComputeGradientHessianInfo;

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
        BaseInfo(FiniteElementElastics::Impl* impl, Float dt)
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
        FiniteElementElastics::Impl* m_impl = nullptr;
        Float                        m_dt;
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
    friend class FiniteElementConstitutionLinearSubsystemReporter;
    friend class FiniteElementConstitutionLineSearchSubreporter;

    friend class FiniteElementMethod;
    void init();  // only be called by FiniteElementMethod

    virtual void do_build() override final;

    friend class FiniteElementElastics;
    void report_extent(ReportExtentInfo& info);
    void compute_energy(ComputeEnergyInfo& info);
    void compute_gradient_hessian(ComputeGradientHessianInfo& info);

    Impl  m_impl;
    SizeT m_index = 0;
};
}  // namespace uipc::backend::cuda

#include "details/finite_element_extra_constitution.inl"