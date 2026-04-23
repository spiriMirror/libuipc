#pragma once
#include <affine_body/inter_affine_body_constitution_manager.h>

namespace uipc::backend::cuda
{
class InterAffineBodyConstitution : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    class BuildInfo
    {
      public:
    };

    using InterGeoInfo = InterAffineBodyConstitutionManager::InterGeoInfo;
    using FilteredInfo = InterAffineBodyConstitutionManager::FilteredInfo;
    using EnergyExtentInfo = InterAffineBodyConstitutionManager::EnergyExtentInfo;
    using ComputeEnergyInfo = InterAffineBodyConstitutionManager::EnergyInfo;
    using GradientHessianExtentInfo =
        InterAffineBodyConstitutionManager::GradientHessianExtentInfo;
    using ComputeGradientHessianInfo = InterAffineBodyConstitutionManager::GradientHessianInfo;
    using TopoReportExtentInfo = InterAffineBodyConstitutionManager::TopoReportExtentInfo;
    using TopoReportInfo       = InterAffineBodyConstitutionManager::TopoReportInfo;

    U64 uid() const noexcept;

  protected:
    virtual void do_build(BuildInfo& info)   = 0;
    virtual void do_init(FilteredInfo& info) = 0;

    virtual void do_report_energy_extent(EnergyExtentInfo& info) = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info)      = 0;

    virtual void do_report_gradient_hessian_extent(GradientHessianExtentInfo& info) = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;

    // Topo reporting — default no-op; joint constitutions override to supply body-pair edges.
    virtual void do_report_topo_extent(TopoReportExtentInfo& info) {}
    virtual void do_report_topo(TopoReportInfo& info)              {}

    virtual U64 get_uid() const noexcept = 0;  // unique identifier for this constitution

    span<const InterGeoInfo> inter_geo_info() const noexcept;

    template <typename ForEachGeometry>
    void for_each(span<S<geometry::GeometrySlot>> geo_slots, ForEachGeometry&& for_every_geometry)
    {
        InterAffineBodyConstitutionManager::_for_each(
            geo_slots, inter_geo_info(), std::forward<ForEachGeometry>(for_every_geometry));
    }

  private:
    friend class InterAffineBodyConstitutionManager;
    virtual void do_build() override;

    void init(FilteredInfo& info);
    void report_energy_extent(EnergyExtentInfo& info);
    void compute_energy(ComputeEnergyInfo& info);

    void report_gradient_hessian_extent(GradientHessianExtentInfo& info);
    void compute_gradient_hessian(ComputeGradientHessianInfo& info);

    void report_topo_extent(TopoReportExtentInfo& info);
    void report_topo(TopoReportInfo& info);

    IndexT m_index = -1;  // index in the InterAffineBodyConstitutionManager
    SimSystemSlot<InterAffineBodyConstitutionManager> m_manager;
};
}  // namespace uipc::backend::cuda