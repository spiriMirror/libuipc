#include <finite_element/finite_element_constraint.h>
#include <uipc/builtin/attribute_name.h>
namespace uipc::backend::cuda
{
class SoftPositionConstraint final : public FiniteElementConstraint
{
    static constexpr U64 SoftPositionConstraintUID = 14ull;

  public:
    using FiniteElementConstraint::FiniteElementConstraint;

    vector<Vector3> h_aim_positions;
    vector<Float>   h_strength_ratio;

    muda::DeviceBuffer<Vector3> aim_positions;
    muda::DeviceBuffer<Float>   strength_ratio;

    void do_build(BuildInfo& info) override {}

    U64 get_uid() const noexcept override { return SoftPositionConstraintUID; }

    // TODO: optimize this
    void do_init(FiniteElementAnimator::FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();
        auto count     = info.anim_indices().size();

        h_aim_positions.resize(count);
        h_strength_ratio.resize(count);

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            {
                auto constraint_strength = sc.vertices().find<Float>("constraint_strength");
                return constraint_strength->view();
            },
            [&](SizeT i, auto strength) { h_strength_ratio[i] = strength; });

        aim_positions.resize(h_aim_positions.size());
        strength_ratio.resize(h_strength_ratio.size());
    }

    void do_step(FiniteElementAnimator::FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            {
                auto aim_pos = sc.vertices().find<Vector3>(builtin::aim_position);
                return aim_pos->view();
            },
            [&](SizeT i, auto position) { h_aim_positions[i] = position; });

        aim_positions.view().copy_from(h_aim_positions.data());
        strength_ratio.view().copy_from(h_strength_ratio.data());
    }

    void do_compute_energy(FiniteElementAnimator::ComputeEnergyInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(info.anim_indices().size(),
                   [indices = info.anim_indices().viewer().name("indices"),
                    xs      = info.xs().viewer().name("xs"),
                    aim_positions = aim_positions.viewer().name("aim_positions"),
                    strength_ratio = strength_ratio.viewer().name("strength_ratio"),
                    masses   = info.masses().viewer().name("masses"),
                    energies = info.energies().viewer().name("energies"),
                    is_fixed = info.is_fixed().viewer().name("is_fixed")] __device__(int I)
                   {
                       auto i = indices(I);

                       if(is_fixed(i) == FiniteElementMethod::FixType::Animated)
                       {
                           auto x = xs(i);
                           auto m = masses(i);
                           auto s = strength_ratio(I);

                           Vector3 dx  = x - aim_positions(I);
                           Float   E   = 0.5f * s * m * dx.dot(dx);
                           energies(i) = E;
                       }
                   });
    }

    void do_compute_gradient_hessian(FiniteElementAnimator::ComputeGradientHessianInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(info.anim_indices().size(),
                   [indices = info.anim_indices().viewer().name("indices"),
                    xs      = info.xs().viewer().name("xs"),
                    aim_positions = aim_positions.viewer().name("aim_positions"),
                    strength_ratio = strength_ratio.viewer().name("strength_ratio"),
                    masses    = info.masses().viewer().name("masses"),
                    gradients = info.gradients().viewer().name("gradients"),
                    hessians  = info.hessians().viewer().name("hessians"),
                    is_fixed = info.is_fixed().viewer().name("is_fixed")] __device__(int I)
                   {
                       auto i = indices(I);

                       if(is_fixed(i) == FiniteElementMethod::FixType::Animated)
                       {
                           auto x = xs(i);
                           auto m = masses(i);
                           auto s = strength_ratio(I);

                           Vector3 dx   = x - aim_positions(I);
                           Vector3 g    = s * m * dx;
                           gradients(i) = g;

                           Matrix3x3 H = s * m * Matrix3x3::Identity();
                           hessians(i) = H;
                       }
                   });
    }
};

REGISTER_SIM_SYSTEM(SoftPositionConstraint);
}  // namespace uipc::backend::cuda
