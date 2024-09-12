#include <finite_element/finite_element_constraint.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
class SoftPositionConstraint final : public FiniteElementConstraint
{
    static constexpr U64 SoftPositionConstraintUID = 14ull;

  public:
    using FiniteElementConstraint::FiniteElementConstraint;

    vector<IndexT>  h_constrained_vertices;
    vector<Vector3> h_aim_positions;
    vector<Float>   h_strength_ratio;

    muda::DeviceBuffer<IndexT> constrained_vertices;

    muda::DeviceBuffer<Vector3> aim_positions;
    muda::DeviceBuffer<Float>   strength_ratio;

    void do_build(BuildInfo& info) override {}

    U64 get_uid() const noexcept override { return SoftPositionConstraintUID; }

    void do_init(FiniteElementAnimator::FilteredInfo& info) override
    {
        auto count = info.anim_vertex_count();

        // reserve memory
        h_constrained_vertices.reserve(count);
        h_aim_positions.reserve(count);
        h_strength_ratio.reserve(count);
        constrained_vertices.reserve(count);
        aim_positions.reserve(count);
        strength_ratio.reserve(count);

        do_step(info);  // do the same thing as do_step
    }

    void do_step(FiniteElementAnimator::FilteredInfo& info) override
    {
        auto geo_slots = world().scene().geometries();

        // clear
        h_constrained_vertices.clear();
        h_aim_positions.clear();
        h_strength_ratio.clear();

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
                auto strength = sc.vertices().find<Float>("strength_ratio");

                return zip(is_constrained->view(), aim_pos->view(), strength->view());
            },
            [&](SizeT vi, auto&& values)
            {
                auto&& [is_constrained, aim_pos, strength] = values;

                if(is_constrained)
                {
                    h_constrained_vertices.push_back(current_vertex_offset + vi);
                    h_aim_positions.push_back(aim_pos);
                    h_strength_ratio.push_back(strength);
                }
            });

        constrained_vertices.resize(h_constrained_vertices.size());
        constrained_vertices.view().copy_from(h_constrained_vertices.data());

        aim_positions.resize(h_aim_positions.size());
        aim_positions.view().copy_from(h_aim_positions.data());

        strength_ratio.resize(h_strength_ratio.size());
        strength_ratio.view().copy_from(h_strength_ratio.data());
    }

    void do_report_extent(FiniteElementAnimator::ReportExtentInfo& info) override
    {
        info.energy_count(h_constrained_vertices.size());
        info.gradient_segment_count(h_constrained_vertices.size());
        info.hessian_block_count(h_constrained_vertices.size());
    }

    void do_compute_energy(FiniteElementAnimator::ComputeEnergyInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(constrained_vertices.size(),
                   [indices = constrained_vertices.viewer().name("indices"),
                    xs      = info.xs().viewer().name("xs"),
                    aim_positions = aim_positions.viewer().name("aim_positions"),
                    strength_ratio = strength_ratio.viewer().name("strength_ratio"),
                    masses   = info.masses().viewer().name("masses"),
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
                           auto    x  = xs(i);
                           auto    m  = masses(i);
                           auto    s  = strength_ratio(I);
                           Vector3 dx = x - aim_positions(I);

                           E = 0.5 * s * m * dx.dot(dx);
                       }
                   });
    }

    void do_compute_gradient_hessian(FiniteElementAnimator::ComputeGradientHessianInfo& info) override
    {
        using namespace muda;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(constrained_vertices.size(),
                   [indices = constrained_vertices.viewer().name("indices"),
                    xs      = info.xs().viewer().name("xs"),
                    aim_positions = aim_positions.viewer().name("aim_positions"),
                    strength_ratio = strength_ratio.viewer().name("strength_ratio"),
                    masses    = info.masses().viewer().name("masses"),
                    gradients = info.gradients().viewer().name("gradients"),
                    hessians  = info.hessians().viewer().name("hessians"),
                    is_fixed = info.is_fixed().viewer().name("is_fixed")] __device__(int I) mutable
                   {
                       auto      i = indices(I);
                       Vector3   G;
                       Matrix3x3 H;
                       if(is_fixed(i))
                       {
                           G = Vector3::Zero();
                           H = Matrix3x3::Zero();
                       }
                       else
                       {
                           auto    x  = xs(i);
                           auto    m  = masses(i);
                           auto    s  = strength_ratio(I);
                           Vector3 dx = x - aim_positions(I);

                           G = s * m * dx;
                           H = s * m * Matrix3x3::Identity();
                       }

                       gradients(I).write(i, G);
                       hessians(I).write(i, i, H);
                   });
    }
};

REGISTER_SIM_SYSTEM(SoftPositionConstraint);
}  // namespace uipc::backend::cuda
