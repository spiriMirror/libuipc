#include <affine_body/affine_body_constraint.h>
#include <uipc/builtin/attribute_name.h>


namespace uipc::backend::cuda
{
// TODO: move to a common header later
inline UIPC_GENERIC Vector12 q_from_transform(const Matrix4x4& trans)
{
    Vector12 q;
    q.segment<3>(0) = trans.block<3, 1>(0, 3);
    q.segment<3>(3) = trans.block<1, 3>(0, 0).transpose();
    q.segment<3>(6) = trans.block<1, 3>(1, 0).transpose();
    q.segment<3>(9) = trans.block<1, 3>(2, 0).transpose();
    return q;
}
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
class SoftTransformConstraint final : public AffineBodyConstraint
{
    static constexpr U64 SoftTransformConstraintUID = 15ull;

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
        auto count = info.anim_body_count();

        // reserve memory
        h_constrained_bodies.reserve(count);
        h_aim_transforms.reserve(count);
        h_strength_ratios.reserve(count);
        constrained_bodies.reserve(count);
        aim_transforms.reserve(count);
        strength_ratios.reserve(count);

        do_step(info);  // do the same thing as do_step
    }

    void do_step(AffineBodyAnimator::FilteredInfo& info) override
    {
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
            [&](SizeT bi, auto&& values)
            {
                auto&& [is_constrained, aim_transform, strength_ratio] = values;

                if(is_constrained)
                {
                    h_constrained_bodies.push_back(current_body_offset + bi);
                    h_aim_transforms.push_back(q_from_transform(aim_transform));
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

    void do_compute_energy(AffineBodyAnimator::ComputeEnergyInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(constrained_bodies.size(),
                   [indices = constrained_bodies.viewer().name("indices"),
                    qs      = info.qs().viewer().name("qs"),
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
                           Vector12 q     = qs(i);
                           Vector12 q_aim = aim_transforms(I);
                           Vector12 dq    = q - q_aim;
                           Vector2  s     = strength_ratios(I);

                           Float translation_strength = s(0);
                           Float rotation_strength    = s(1);

                           Matrix12x12 M = body_masses(i).to_mat();

                           M.block<3, 3>(0, 0) *= translation_strength;
                           M.block<9, 9>(3, 3) *= rotation_strength;

                           E = 0.5 * dq.transpose() * M * dq;
                       }
                   });
    }

    void do_compute_gradient_hessian(AffineBodyAnimator::ComputeGradientHessianInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(constrained_bodies.size(),
                   [indices = constrained_bodies.viewer().name("indices"),
                    qs      = info.qs().viewer().name("qs"),
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
                           Vector12 q     = qs(i);
                           Vector12 q_aim = aim_transforms(I);
                           Vector12 dq    = q - q_aim;
                           Vector2  s     = strength_ratios(I);

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