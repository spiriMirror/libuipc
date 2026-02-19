#include <finite_element/finite_element_animator.h>
#include <finite_element/finite_element_constraint.h>
#include <finite_element/fem_line_search_subreporter.h>
#include <finite_element/fem_linear_subsystem_reporter.h>
#include <uipc/builtin/attribute_name.h>
#include <utils/report_extent_check.h>
#include <uipc/common/enumerate.h>
#include <muda/cub/device/device_reduce.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FiniteElementAnimator);

void FiniteElementAnimator::do_build(BuildInfo& info)
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();
    m_impl.global_animator       = &require<GlobalAnimator>();
}

void FiniteElementAnimator::add_constraint(FiniteElementConstraint* constraint)
{
    m_impl.constraints.register_sim_system(*constraint);
}

void FiniteElementAnimator::assemble(FEMLinearSubsystemReporter::AssembleInfo& info)
{
    // compute the gradient and hessian
    for(auto constraint : m_impl.constraints.view())
    {
        ComputeGradientHessianInfo this_info{
            &m_impl,
            constraint->m_index,
            info.dt(),
            info.gradients(),
            info.hessians(),
            info.gradient_only()};
        constraint->compute_gradient_hessian(this_info);
    }
}

void FiniteElementAnimator::do_init()
{
    m_impl.init(world());
}

void FiniteElementAnimator::do_step()
{
    m_impl.step();
}

void FiniteElementAnimator::Impl::init(backend::WorldVisitor& world)
{
    // sort the constraints by uid
    auto constraint_view = constraints.view();

    std::sort(constraint_view.begin(),
              constraint_view.end(),
              [](const FiniteElementConstraint* a, const FiniteElementConstraint* b)
              { return a->uid() < b->uid(); });

    // setup constraint index and the mapping from uid to index
    for(auto&& [i, constraint] : enumerate(constraint_view))
    {
        auto uid                     = constraint->uid();
        uid_to_constraint_index[uid] = i;
        constraint->m_index          = i;
    }

    constraint_geo_info_offsets_counts.resize(constraint_view.size());

    auto        geo_slots = world.scene().geometries();
    const auto& geo_infos = finite_element_method->m_impl.geo_infos;

    vector<list<AnimatedGeoInfo>> constraint_geo_info_buffer(constraint_view.size());
    span<IndexT> constraint_geo_info_counts = constraint_geo_info_offsets_counts.counts();

    for(auto& info : geo_infos)
    {
        auto  geo_slot = geo_slots[info.geo_slot_index];
        auto& geo      = geo_slot->geometry();
        auto  uids     = geo.meta().find<VectorXu64>(builtin::constraint_uids);
        if(uids)
        {
            const auto& uid_values = uids->view().front();

            for(auto&& uid_value : uid_values)
            {
                auto it = uid_to_constraint_index.find(uid_value);
                UIPC_ASSERT(it != uid_to_constraint_index.end(),
                            "FiniteElementAnimator: No responsible backend SimSystem registered for constraint uid {}",
                            uid_value);
                auto index = it->second;
                constraint_geo_info_buffer[index].push_back(info);
            }
        }
    }

    // fill the counts
    std::ranges::transform(constraint_geo_info_buffer,
                           constraint_geo_info_counts.begin(),
                           [](const auto& infos)
                           { return static_cast<IndexT>(infos.size()); });


    // scan to get offsets and total count
    constraint_geo_info_offsets_counts.scan();

    auto total_anim_geo_info = constraint_geo_info_offsets_counts.total_count();
    anim_geo_infos.reserve(total_anim_geo_info);

    for(auto& infos : constraint_geo_info_buffer)
    {
        for(auto& info : infos)
        {
            anim_geo_infos.push_back(info);
        }
    }

    // initialize the constraints
    for(auto constraint : constraint_view)
    {
        FilteredInfo info{this, constraint->m_index};
        constraint->init(info);
    }

    constraint_energy_offsets_counts.resize(constraint_view.size());
    constraint_gradient_offsets_counts.resize(constraint_view.size());
    constraint_hessian_offsets_counts.resize(constraint_view.size());
}

void FiniteElementAnimator::Impl::step()
{
    for(auto constraint : constraints.view())
    {
        FilteredInfo info{this, constraint->m_index};
        constraint->step(info);
    }

    span<IndexT> constraint_energy_counts = constraint_energy_offsets_counts.counts();
    span<IndexT> constraint_gradient_counts = constraint_gradient_offsets_counts.counts();
    span<IndexT> constraint_hessian_counts = constraint_hessian_offsets_counts.counts();

    for(auto&& [i, constraint] : enumerate(constraints.view()))
    {
        ReportExtentInfo this_info;
        constraint->report_extent(this_info);

        constraint_energy_counts[i]   = this_info.m_energy_count;
        constraint_gradient_counts[i] = this_info.m_gradient_segment_count;
        constraint_hessian_counts[i]  = this_info.m_hessian_block_count;
    }

    // update the offsets
    constraint_energy_offsets_counts.scan();
    constraint_gradient_offsets_counts.scan();
    constraint_hessian_offsets_counts.scan();
}

void FiniteElementAnimator::compute_energy(FEMLineSearchSubreporter::ComputeEnergyInfo& info)
{
    using namespace muda;
    for(auto constraint : m_impl.constraints.view())
    {
        ComputeEnergyInfo this_info{&m_impl, constraint->m_index, info.dt(), info.energies()};
        constraint->compute_energy(this_info);
    }
}

auto FiniteElementAnimator::FilteredInfo::anim_geo_infos() const -> span<const AnimatedGeoInfo>
{
    auto [offset, count] = m_impl->constraint_geo_info_offsets_counts[m_index];
    return span<const AnimatedGeoInfo>{m_impl->anim_geo_infos}.subspan(offset, count);
}

Float FiniteElementAnimator::BaseInfo::substep_ratio() const noexcept
{
    return m_impl->global_animator->substep_ratio();
}

muda::CBufferView<Vector3> FiniteElementAnimator::BaseInfo::xs() const noexcept
{
    return m_impl->finite_element_method->xs();
}

muda::CBufferView<Vector3> FiniteElementAnimator::BaseInfo::x_prevs() const noexcept
{
    return m_impl->finite_element_method->x_prevs();
}

muda::CBufferView<Float> FiniteElementAnimator::BaseInfo::masses() const noexcept
{
    return m_impl->finite_element_method->masses();
}

muda::CBufferView<IndexT> FiniteElementAnimator::BaseInfo::is_fixed() const noexcept
{
    return m_impl->finite_element_method->is_fixed();
}

muda::BufferView<Float> FiniteElementAnimator::ComputeEnergyInfo::energies() const noexcept
{
    auto [offset, count] = m_impl->constraint_energy_offsets_counts[m_index];
    return m_energies.subview(offset, count);
}

muda::DoubletVectorView<Float, 3> FiniteElementAnimator::ComputeGradientHessianInfo::gradients() const noexcept
{
    auto [offset, count] = m_impl->constraint_gradient_offsets_counts[m_index];
    return m_gradients.subview(offset, count);
}

muda::TripletMatrixView<Float, 3> FiniteElementAnimator::ComputeGradientHessianInfo::hessians() const noexcept
{
    auto [offset, count] = m_impl->constraint_hessian_offsets_counts[m_index];
    return m_hessians.subview(offset, count);
}

bool FiniteElementAnimator::ComputeGradientHessianInfo::gradient_only() const noexcept
{
    return m_gradient_only;
}

void FiniteElementAnimator::ReportExtentInfo::hessian_count(SizeT count) noexcept
{
    m_hessian_block_count = count;
}
void FiniteElementAnimator::ReportExtentInfo::gradient_count(SizeT count) noexcept
{
    m_gradient_segment_count = count;
}
void FiniteElementAnimator::ReportExtentInfo::energy_count(SizeT count) noexcept
{
    m_energy_count = count;
}

void FiniteElementAnimator::ReportExtentInfo::check(std::string_view name) const
{
    check_report_extent(m_gradient_only_checked, m_gradient_only, m_hessian_block_count, name);
}
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
class FiniteElementAnimatorLinearSubsystemReporter final : public FEMLinearSubsystemReporter
{
  public:
    using FEMLinearSubsystemReporter::FEMLinearSubsystemReporter;

    SimSystemSlot<FiniteElementAnimator> animator;

    virtual void do_build(BuildInfo& info) override
    {
        animator = require<FiniteElementAnimator>();
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_report_extent(ReportExtentInfo& info) override
    {
        SizeT gradient_count =
            animator->m_impl.constraint_gradient_offsets_counts.total_count();
        info.gradient_count(gradient_count);

        SizeT hessian_count = 0;
        if(!info.gradient_only())
            hessian_count =
                animator->m_impl.constraint_hessian_offsets_counts.total_count();
        info.hessian_count(hessian_count);
    }

    virtual void do_assemble(AssembleInfo& info) override
    {
        animator->assemble(info);
    }
};

REGISTER_SIM_SYSTEM(FiniteElementAnimatorLinearSubsystemReporter);

class FiniteElementAnimatorLineSearchSubreporter final : public FEMLineSearchSubreporter
{
  public:
    using FEMLineSearchSubreporter::FEMLineSearchSubreporter;
    SimSystemSlot<FiniteElementAnimator> animator;

    virtual void do_build(BuildInfo& info) override
    {
        animator = require<FiniteElementAnimator>();
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_report_extent(ReportExtentInfo& info) override
    {
        SizeT energy_count =
            animator->m_impl.constraint_energy_offsets_counts.total_count();
        info.energy_count(energy_count);
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        animator->compute_energy(info);
    }
};

REGISTER_SIM_SYSTEM(FiniteElementAnimatorLineSearchSubreporter);
}  // namespace uipc::backend::cuda
