#pragma once
#include <sim_system.h>
#include <finite_element/finite_element_method.h>

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

        SizeT stencil_dim = 0;

        SizeT energy_offset = 0;
        SizeT energy_count  = 0;

        SizeT gradient_offset = 0;
        SizeT gradient_count  = 0;

        SizeT hessian_offset = 0;
        SizeT hessian_count  = 0;

        FiniteElementMethod*                 finite_element_method = nullptr;
        vector<FiniteElementMethod::GeoInfo> geo_infos;

        FiniteElementMethod::Impl& fem() noexcept
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
                      ForEach&&                       for_each_action) noexcept;

      private:
        Impl* m_impl = nullptr;
    };

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl, Float dt)
            : m_impl(impl)
        {
        }

        Float dt() const noexcept;

        muda::CBufferView<Vector3> xs() const noexcept;
        muda::CBufferView<Vector3> x_bars() const noexcept;
        muda::CBufferView<IndexT>  is_fixed() const noexcept;

      protected:
        Impl* m_impl = nullptr;
        Float m_dt;
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
        friend class FiniteElementExtraConstitution;
        SizeT m_energy_count = 0;
        SizeT m_stencil_dim  = 0;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
        muda::BufferView<Float> energies() const noexcept;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
        muda::DoubletVectorView<Float, 3> gradients() const noexcept;
        muda::TripletMatrixView<Float, 3> hessians() const noexcept;
    };

    U64 uid() const noexcept;

  protected:
    virtual void do_build(BuildInfo& info)                  = 0;
    virtual U64  get_uid() const noexcept                   = 0;
    virtual void do_init(FilteredInfo& info)                = 0;
    virtual void do_report_extent(ReportExtentInfo& info)   = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;

    span<const FiniteElementMethod::GeoInfo> geo_infos() const noexcept;

  private:
    friend class FiniteElementMethod;
    void init();                 // only be called by FiniteElementMethod
    void collect_extent_info();  // only be called by FiniteElementMethod
    void compute_energy(FiniteElementMethod::ComputeExtraEnergyInfo& info);  // only be called by FiniteElementMethod
    friend class FEMGradientHessianComputer;
    void compute_gradient_hessian(FiniteElementMethod::ComputeExtraGradientHessianInfo& info);  // only be called by FiniteElementMethod

    virtual void do_build() override final;
    Impl         m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/finite_element_extra_constitution.inl"