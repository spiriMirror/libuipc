#include <affine_body/affine_body_constraint.h>
#include <affine_body/utils.h>
#include <uipc/builtin/attribute_name.h>
#include <kernel_cout.h>
#include <animator/utils.h>

namespace uipc::backend::cuda
{
class SoftTransformConstraint final : public AffineBodyConstraint
{
    static constexpr U64 SoftTransformConstraintUID = 16ull;

  public:
    using AffineBodyConstraint::AffineBodyConstraint;

    vector<IndexT>   h_constrained_bodies;
    vector<Vector12> h_aim_transforms;
    vector<Vector2>  h_strength_ratios;

    muda::DeviceBuffer<IndexT>   constrained_bodies;
    muda::DeviceBuffer<Vector12> aim_transforms;
    muda::DeviceBuffer<Vector2>  strength_ratios;

    virtual void do_build(BuildInfo& info) override {}

    virtual U64 get_uid() const noexcept override
    {
        return SoftTransformConstraintUID;
    }

    void do_init(AffineBodyAnimator::FilteredInfo& info) override
    {
        do_step(info);  // do the same thing as do_step
    }

    void do_step(AffineBodyAnimator::FilteredInfo& info) override
    {
        using ForEachInfo = AffineBodyDynamics::ForEachInfo;

        auto geo_slots = world().scene().geometries();

        // clear
        h_constrained_bodies.clear();
        h_aim_transforms.clear();
        h_strength_ratios.clear();

        IndexT current_body_offset = 0;
        info.for_each(
            geo_slots,
            [&](geometry::SimplicialComplex& sc)
            {
                auto body_offset = sc.meta().find<IndexT>(builtin::backend_abd_body_offset);
                current_body_offset = body_offset->view().front();

                auto is_constrained = sc.instances().find<IndexT>(builtin::is_constrained);
                auto aim_transform = sc.instances().find<Matrix4x4>(builtin::aim_transform);
                auto strength_ratio = sc.instances().find<Vector2>("strength_ratio");

                return zip(is_constrained->view(),
                           aim_transform->view(),
                           strength_ratio->view());
            },
            [&](const ForEachInfo& I, auto&& values)
            {
                SizeT bI = I.local_index() + current_body_offset;

                auto&& [is_constrained, aim_transform, strength_ratio] = values;

                if(is_constrained)
                {
                    h_constrained_bodies.push_back(bI);
                    Vector12 q = transform_to_q(aim_transform);
                    h_aim_transforms.push_back(q);
                    h_strength_ratios.push_back(strength_ratio);
                }
            });

        constrained_bodies.resize(h_constrained_bodies.size());
        constrained_bodies.view().copy_from(h_constrained_bodies.data());

        aim_transforms.resize(h_aim_transforms.size());
        aim_transforms.view().copy_from(h_aim_transforms.data());

        strength_ratios.resize(h_strength_ratios.size());
        strength_ratios.view().copy_from(h_strength_ratios.data());
    }

    void do_report_extent(AffineBodyAnimator::ReportExtentInfo& info) override
    {
        info.energy_count(h_constrained_bodies.size());
        info.gradient_segment_count(h_constrained_bodies.size());
        info.hessian_block_count(h_constrained_bodies.size());
    }

    void do_compute_energy(AffineBodyAnimator::EnergyInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(constrained_bodies.size(),
                   [substep_ratio = info.substep_ratio(),
                    indices       = constrained_bodies.viewer().name("indices"),
                    qs            = info.qs().viewer().name("qs"),
                    q_prevs       = info.q_prevs().viewer().name("q_prevs"),
                    aim_transforms = aim_transforms.viewer().name("aim_transforms"),
                    strength_ratios = strength_ratios.viewer().name("strength_ratios"),
                    body_masses = info.body_masses().viewer().name("body_masses"),
                    energies = info.energies().viewer().name("energies"),
                    is_fixed = info.is_fixed().viewer().name("is_fixed")] __device__(int I)
                   {
                       auto  i = indices(I);
                       auto& E = energies(I);

                       if(is_fixed(i))
                       {
                           E = 0.0;
                       }
                       else
                       {
                           Vector12 q      = qs(i);
                           Vector12 q_prev = q_prevs(i);
                           Vector12 q_aim = lerp(q_prev, aim_transforms(I), substep_ratio);
                           Vector12 dq = q - q_aim;
                           Vector2  s  = strength_ratios(I);

                           Float translation_strength = s(0);
                           Float rotation_strength    = s(1);

                           Matrix12x12 M = body_masses(i).to_mat();

                           M.block<3, 3>(0, 0) *= translation_strength;
                           M.block<9, 9>(3, 3) *= rotation_strength;

                           E = 0.5 * dq.transpose() * M * dq;
                       }
                   });
    }

    void do_compute_gradient_hessian(AffineBodyAnimator::GradientHessianInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(constrained_bodies.size(),
                   [substep_ratio = info.substep_ratio(),
                    indices       = constrained_bodies.viewer().name("indices"),
                    qs            = info.qs().viewer().name("qs"),
                    q_prevs       = info.q_prevs().viewer().name("q_prevs"),
                    aim_transforms = aim_transforms.viewer().name("aim_transforms"),
                    strength_ratios = strength_ratios.viewer().name("strength_ratios"),
                    body_masses = info.body_masses().viewer().name("body_masses"),
                    gradients = info.gradients().viewer().name("gradients"),
                    hessians  = info.hessians().viewer().name("hessians"),
                    is_fixed = info.is_fixed().viewer().name("is_fixed")] __device__(int I) mutable
                   {
                       auto i = indices(I);

                       Vector12    G;
                       Matrix12x12 H;

                       if(is_fixed(i))
                       {
                           G.setZero();
                           H.setZero();
                       }
                       else
                       {
                           Vector12 q      = qs(i);
                           Vector12 q_prev = q_prevs(i);
                           Vector12 q_aim = lerp(q_prev, aim_transforms(I), substep_ratio);
                           Vector12 dq = q - q_aim;
                           Vector2  s  = strength_ratios(I);

                           Float translation_strength = s(0);
                           Float rotation_strength    = s(1);

                           Matrix12x12 M = body_masses(i).to_mat();

                           M.block<3, 3>(0, 0) *= translation_strength;
                           M.block<9, 9>(3, 3) *= rotation_strength;

                           G = M * dq;
                           H = M;
                       }

                       gradients(I).write(i, G);
                       hessians(I).write(i, i, H);
                   });
    }
};

REGISTER_SIM_SYSTEM(SoftTransformConstraint);
}  // namespace uipc::backend::cuda