#include <finite_element/finite_element_constraint.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void SoftPositionConstraint_do_compute_energy_kernel(
        Float                           substep_ratio,
        cuda_tool::BufferView<IndexT>   indices,
        cuda_tool::CBufferView<Vector3> xs,
        cuda_tool::CBufferView<Vector3> x_prevs,
        cuda_tool::BufferView<Vector3>  aim_positions,
        cuda_tool::BufferView<Float>    strength_ratio,
        cuda_tool::CBufferView<Float>   masses,
        cuda_tool::BufferView<Float>    energies,
        cuda_tool::CBufferView<IndexT>  is_fixed,
        int                             n)
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
            Vector3 x      = xs(i);
            Vector3 x_prev = x_prevs(i);
            Vector3 aim_x  = lerp(x_prev, aim_positions(I), substep_ratio);
            Float   m      = masses(i);
            Float   s      = strength_ratio(I);
            Vector3 dx     = x - aim_x;

            E = 0.5 * s * m * dx.dot(dx);
        }
    }

    __global__ void SoftPositionConstraint_do_compute_gradient_hessian_kernel(
        Float                                  substep_ratio,
        cuda_tool::BufferView<IndexT>          indices,
        cuda_tool::CBufferView<Vector3>        xs,
        cuda_tool::CBufferView<Vector3>        x_prevs,
        cuda_tool::BufferView<Vector3>         aim_positions,
        cuda_tool::BufferView<Float>           strength_ratio,
        cuda_tool::CBufferView<Float>          masses,
        cuda_tool::DoubletVectorView<Float, 3> gradients,
        cuda_tool::TripletMatrixView<Float, 3> hessians,
        cuda_tool::CBufferView<IndexT>         is_fixed,
        bool                                   gradient_only,
        int                                    n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto    i = indices(I);
        Vector3 G;
        Float   m = 0.0;
        Float   s = 0.0;
        if(is_fixed(i))
        {
            G = Vector3::Zero();
        }
        else
        {
            Vector3 x      = xs(i);
            Vector3 x_prev = x_prevs(i);
            Vector3 aim_x  = lerp(x_prev, aim_positions(I), substep_ratio);
            m              = masses(i);
            s              = strength_ratio(I);
            Vector3 dx     = x - aim_x;

            G = s * m * dx;
        }

        gradients(I).write(i, G);

        if(gradient_only)
            return;

        Matrix3x3 H = s * m * Matrix3x3::Identity();
        if(is_fixed(i))
            H = Matrix3x3::Zero();
        hessians(I).write(i, i, H);
    }
}  // namespace

class SoftPositionConstraint final : public FiniteElementConstraint
{
    static constexpr U64 SoftPositionConstraintUID = 14ull;

  public:
    using FiniteElementConstraint::FiniteElementConstraint;

    vector<IndexT>  h_constrained_vertices;
    vector<Vector3> h_aim_positions;
    vector<Float>   h_strength_ratios;

    cuda_tool::DeviceBuffer<IndexT>  constrained_vertices;
    cuda_tool::DeviceBuffer<Vector3> aim_positions;
    cuda_tool::DeviceBuffer<Float>   strength_ratios;

    void do_build(BuildInfo& info) override {}

    U64 get_uid() const noexcept override { return SoftPositionConstraintUID; }

    void do_init(FiniteElementAnimator::FilteredInfo& info) override
    {
        do_step(info);  // do the same thing as do_step
    }

    void do_step(FiniteElementAnimator::FilteredInfo& info) override
    {
        using ForEachInfo = FiniteElementMethod::ForEachInfo;

        auto geo_slots = world().scene().geometries();

        // clear
        h_constrained_vertices.clear();
        h_aim_positions.clear();
        h_strength_ratios.clear();

        IndexT current_vertex_offset = 0;
        info.for_each(
            geo_slots,
            [&](geometry::SimplicialComplex& sc)
            {
                auto vertex_offset =
                    sc.meta().find<IndexT>(builtin::backend_fem_vertex_offset);
                current_vertex_offset = vertex_offset->view().front();

                auto is_constrained = sc.vertices().find<IndexT>(builtin::is_constrained);
                auto aim_pos = sc.vertices().find<Vector3>(builtin::aim_position);
                auto strength_ratio = sc.vertices().find<Float>("strength_ratio");

                return zip(is_constrained->view(),
                           aim_pos->view(),
                           strength_ratio->view());
            },
            [&](const ForEachInfo& I, auto&& values)
            {
                auto vI = I.local_index() + current_vertex_offset;

                auto&& [is_constrained, aim_pos, strength] = values;

                if(is_constrained)
                {
                    h_constrained_vertices.push_back(vI);
                    h_aim_positions.push_back(aim_pos);
                    h_strength_ratios.push_back(strength);
                }
            });

        constrained_vertices.resize(h_constrained_vertices.size());
        constrained_vertices.view().copy_from(h_constrained_vertices.data());

        aim_positions.resize(h_aim_positions.size());
        aim_positions.view().copy_from(h_aim_positions.data());

        strength_ratios.resize(h_strength_ratios.size());
        strength_ratios.view().copy_from(h_strength_ratios.data());
    }

    void do_report_extent(FiniteElementAnimator::ReportExtentInfo& info) override
    {
        info.energy_count(h_constrained_vertices.size());
        info.gradient_count(h_constrained_vertices.size());

        if(info.gradient_only())
            return;

        info.hessian_count(h_constrained_vertices.size());
    }

    void do_compute_energy(FiniteElementAnimator::ComputeEnergyInfo& info) override
    {
        auto k = SoftPositionConstraint_do_compute_energy_kernel;
        int  n = (int)constrained_vertices.size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.substep_ratio(),
                constrained_vertices.view(),
                info.xs(),
                info.x_prevs(),
                aim_positions.view(),
                strength_ratios.view(),
                info.masses(),
                info.energies(),
                info.is_fixed(),
                n);
        }
    }

    void do_compute_gradient_hessian(FiniteElementAnimator::ComputeGradientHessianInfo& info) override
    {
        auto k = SoftPositionConstraint_do_compute_gradient_hessian_kernel;
        int  n = (int)constrained_vertices.size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.substep_ratio(),
                constrained_vertices.view(),
                info.xs(),
                info.x_prevs(),
                aim_positions.view(),
                strength_ratios.view(),
                info.masses(),
                info.gradients(),
                info.hessians(),
                info.is_fixed(),
                info.gradient_only(),
                n);
        }
    }
};

REGISTER_SIM_SYSTEM(SoftPositionConstraint);
}  // namespace uipc::backend::cuda
