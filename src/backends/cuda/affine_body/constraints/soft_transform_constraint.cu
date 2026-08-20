#include <affine_body/affine_body_constraint.h>
#include <affine_body/utils.h>
#include <uipc/builtin/attribute_name.h>
#include <kernel_cout.h>
#include <animator/utils.h>

namespace uipc::backend::cuda
{
inline UIPC_GENERIC Matrix12x12 compute_constraint_mass(const ABDJacobiDyadicMass& mass,
                                                        Float translation_strength,
                                                        Float rotation_strength)
{
    Float s_t = translation_strength;
    Float s_r = rotation_strength;
    Float m   = mass.mass();

    UIPC_KERNEL_ASSERT(m > 0.0, "ABDJacobiDyadicMass has non-positive mass (%f), cannot build constraint mass matrix.", m);

    Matrix12x12 M = mass.to_mat();

    // Build M_cm = m*J(c)^T*J(c): the ABD mass matrix of a point mass m
    // concentrated at the center of mass c (where mc = m*c is the first moment).
    // M = M_cm + M_rot  (parallel-axis decomposition)
    // M_cm captures CM translation kinetics; M_rot captures rotation/deformation about CM.
    Vector3   mc   = mass.mass_times_x_bar();  // m*c
    Matrix3x3 mccT = mc * mc.transpose() / m;  // m*c*c^T

    ABDJacobiDyadicMass cm_mass = ABDJacobiDyadicMass::from_dyadic_mass(m, mc, mccT);
    Matrix12x12 M_cm = cm_mass.to_mat();

    // M_constraint = s_t*M_cm + s_r*(M - M_cm)
    //              = s_r*M + (s_t - s_r)*M_cm
    return s_r * M + (s_t - s_r) * M_cm;
}

namespace
{
    __global__ void soft_transform_constraint_compute_energy_kernel(
        Float                                       substep_ratio,
        cuda_tool::BufferView<IndexT>               indices,
        cuda_tool::CBufferView<Vector12>            qs,
        cuda_tool::CBufferView<Vector12>            q_prevs,
        cuda_tool::BufferView<Vector12>             aim_transforms,
        cuda_tool::BufferView<Vector2>              strength_ratios,
        cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses,
        cuda_tool::BufferView<Float>                energies,
        cuda_tool::CBufferView<IndexT>              is_fixed,
        int                                         n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
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

            Matrix12x12 M =
                compute_constraint_mass(body_masses(i), s(0), s(1));

            E = 0.5 * dq.transpose() * M * dq;
        }
    }

    __global__ void soft_transform_constraint_compute_gradient_hessian_kernel(
        Float                                       substep_ratio,
        cuda_tool::BufferView<IndexT>               indices,
        cuda_tool::CBufferView<Vector12>            qs,
        cuda_tool::CBufferView<Vector12>            q_prevs,
        cuda_tool::BufferView<Vector12>             aim_transforms,
        cuda_tool::BufferView<Vector2>              strength_ratios,
        cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses,
        cuda_tool::DoubletVectorView<Float, 12>     gradients,
        cuda_tool::TripletMatrixView<Float, 12>     hessians,
        cuda_tool::CBufferView<IndexT>              is_fixed,
        bool                                        gradient_only,
        int                                         n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto i = indices(I);

        Vector12    G;
        Matrix12x12 M;

        if(is_fixed(i))
        {
            G.setZero();
            M.setZero();
        }
        else
        {
            Vector12 q      = qs(i);
            Vector12 q_prev = q_prevs(i);
            Vector12 q_aim = lerp(q_prev, aim_transforms(I), substep_ratio);
            Vector12 dq = q - q_aim;
            Vector2  s  = strength_ratios(I);

            M = compute_constraint_mass(body_masses(i), s(0), s(1));
            G = M * dq;
        }

        gradients(I).write(i, G);

        if(gradient_only)
            return;

        hessians(I).write(i, i, M);
    }
}  // namespace

class SoftTransformConstraint final : public AffineBodyConstraint
{
    static constexpr U64 SoftTransformConstraintUID = 16ull;

  public:
    using AffineBodyConstraint::AffineBodyConstraint;

    vector<IndexT>   h_constrained_bodies;
    vector<Vector12> h_aim_transforms;
    vector<Vector2>  h_strength_ratios;

    cuda_tool::DeviceBuffer<IndexT>   constrained_bodies;
    cuda_tool::DeviceBuffer<Vector12> aim_transforms;
    cuda_tool::DeviceBuffer<Vector2>  strength_ratios;

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
                    UIPC_ASSERT(strength_ratio(0) >= 0.0 && strength_ratio(1) >= 0.0,
                                "Strength ratios must be non-negative, but got ({}, {})",
                                strength_ratio(0),
                                strength_ratio(1));
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
        info.gradient_count(h_constrained_bodies.size());
        if(info.gradient_only())
            return;

        info.hessian_count(h_constrained_bodies.size());
    }

    void do_compute_energy(AffineBodyAnimator::ComputeEnergyInfo& info) override
    {
        int n = (int)constrained_bodies.size();
        if(n > 0)
        {
            auto k = soft_transform_constraint_compute_energy_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.substep_ratio(),
                constrained_bodies.view(),
                info.qs(),
                info.q_prevs(),
                aim_transforms.view(),
                strength_ratios.view(),
                info.body_masses(),
                info.energies(),
                info.is_fixed(),
                n);
        }
    }

    void do_compute_gradient_hessian(AffineBodyAnimator::ComputeGradientHessianInfo& info) override
    {
        int n = (int)constrained_bodies.size();
        if(n > 0)
        {
            auto k = soft_transform_constraint_compute_gradient_hessian_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.substep_ratio(),
                constrained_bodies.view(),
                info.qs(),
                info.q_prevs(),
                aim_transforms.view(),
                strength_ratios.view(),
                info.body_masses(),
                info.gradients(),
                info.hessians(),
                info.is_fixed(),
                info.gradient_only(),
                n);
        }
    }
};

REGISTER_SIM_SYSTEM(SoftTransformConstraint);
}  // namespace uipc::backend::cuda