#include <sim_system.h>
#include <affine_body/affine_body_extra_constitution.h>
#include <affine_body/affine_body_dynamics.h>
#include <uipc/common/enumerate.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
class AffineBodyExternalWrench final : public AffineBodyExtraConstitution
{
  public:
    static constexpr U64 ConstitutionUID = 666;

    using AffineBodyExtraConstitution::AffineBodyExtraConstitution;

    vector<Vector12> h_wrenches;  // external wrenches (12D generalized forces)
    vector<IndexT>   h_body_ids;  // global body IDs for each wrench

    muda::DeviceBuffer<Vector12> wrenches;
    muda::DeviceBuffer<IndexT>   body_ids;

    virtual void do_build(AffineBodyExtraConstitution::BuildInfo& info) override {}

    U64 get_uid() const noexcept override { return ConstitutionUID; }

    void do_init(AffineBodyExtraConstitution::FilteredInfo& info) override
    {
        // Count total bodies
        SizeT total_bodies = 0;
        for(auto&& geo_info : info.geo_infos())
        {
            total_bodies += geo_info.body_count;
        }

        h_wrenches.reserve(total_bodies);
        h_body_ids.reserve(total_bodies);
        wrenches.reserve(total_bodies);
        body_ids.reserve(total_bodies);

        // Read wrenches from scene
        do_step(info);
    }

    void do_step(AffineBodyExtraConstitution::FilteredInfo& info) override
    {
        using ForEachInfo = AffineBodyDynamics::ForEachInfo;

        // Clear old data
        h_wrenches.clear();
        h_body_ids.clear();

        // Re-read external wrenches from scene
        auto geo_slots = world().scene().geometries();

        // For each geometry, collect wrenches and record global body IDs
        for(auto&& geo_info : info.geo_infos())
        {
            auto  geo_slot = geo_slots[geo_info.geo_slot_index];
            auto& geo      = geo_slot->geometry();
            auto* sc       = geo.template as<geometry::SimplicialComplex>();

            auto wrench_attr = sc->instances().find<Vector12>("external_wrench");
            if(!wrench_attr)
                continue;  // Skip if no external_wrench attribute

            auto wrench_view = wrench_attr->view();

            for(auto&& [i, wrench] : enumerate(wrench_view))
            {
                h_wrenches.push_back(wrench);
                h_body_ids.push_back(geo_info.body_offset + i);
            }
        }

        _build_on_device();
    }

    void _build_on_device()
    {
        auto async_copy = []<typename T>(span<T> src, muda::DeviceBuffer<T>& dst)
        {
            muda::BufferLaunch().resize<T>(dst, src.size());
            muda::BufferLaunch().copy<T>(dst.view(), src.data());
        };

        async_copy(span{h_wrenches}, wrenches);
        async_copy(span{h_body_ids}, body_ids);
    }

    virtual void do_compute_energy(AffineBodyExtraConstitution::ComputeEnergyInfo& info) override
    {
        using namespace muda;

        auto wrench_count = wrenches.size();

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(wrench_count,
                   [energies = info.energies().viewer().name("energies"),
                    qs       = info.qs().cviewer().name("qs"),
                    wrenches = wrenches.cviewer().name("wrenches"),
                    body_ids = body_ids.cviewer().name("body_ids"),
                    dt       = info.dt()] __device__(int i) mutable
                   {
                       IndexT      body_id = body_ids(i);
                       const auto& q       = qs(body_id);
                       const auto& W       = wrenches(i);

                       // Energy = -W^T * q * dt^2 (incremental potential for implicit integration)
                       // Full 12D generalized wrench: W = [f; vec(dS/dt)] where f is 3D force
                       Float dt2 = dt * dt;
                       energies(body_id) += -W.dot(q) * dt2;
                   });
    }

    virtual void do_compute_gradient_hessian(AffineBodyExtraConstitution::ComputeGradientHessianInfo& info) override
    {
        using namespace muda;

        auto N = wrenches.size();

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(N,
                   [gradients = info.gradients().viewer().name("gradients"),
                    wrenches  = wrenches.cviewer().name("wrenches"),
                    body_ids  = body_ids.cviewer().name("body_ids"),
                    dt        = info.dt()] __device__(int i) mutable
                   {
                       IndexT      body_id = body_ids(i);
                       const auto& W       = wrenches(i);

                       Float dt2 = dt * dt;

                       // Gradient: dE/dq = -W * dt^2
                       Vector12 G = -W * dt2;

                       // Atomic add to accumulate gradient
                       muda::eigen::atomic_add(gradients(body_id), G);

                       // Hessian: d²E/dq² = 0 (constant wrench, no second derivative)
                       // No need to write zero hessian
                   });
    }
};

REGISTER_SIM_SYSTEM(AffineBodyExternalWrench);
}  // namespace uipc::backend::cuda
