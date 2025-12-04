#include <affine_body/inter_affine_body_animator.h>
#include <affine_body/inter_affine_body_constraint.h>
#include <uipc/builtin/attribute_name.h>
#include <muda/cub/device/device_reduce.h>
#include <affine_body/abd_line_search_reporter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(InterAffineBodyAnimator);

void InterAffineBodyAnimator::do_build(BuildInfo& info)
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
    m_impl.manager         = &require<InterAffineBodyConstitutionManager>();
    m_impl.global_animator = &require<GlobalAnimator>();
    auto dt_attr           = world().scene().config().find<Float>("dt");
    m_impl.dt              = dt_attr->view()[0];
}

void InterAffineBodyAnimator::add_constraint(InterAffineBodyConstraint* constraint)
{
    m_impl.constraints.register_subsystem(*constraint);
}

void InterAffineBodyAnimator::do_init()
{
    m_impl.init(world());
}

void InterAffineBodyAnimator::do_step()
{
    m_impl.step();
}

void InterAffineBodyAnimator::Impl::init(backend::WorldVisitor& world)
{
    // sort the constraints by uid
    auto constraint_view = constraints.view();

    std::sort(constraint_view.begin(),
              constraint_view.end(),
              [](const InterAffineBodyConstraint* a, const InterAffineBodyConstraint* b)
              { return a->uid() < b->uid(); });

    // setup constraint index and the mapping from uid to index
    for(auto&& [i, constraint] : enumerate(constraint_view))
    {
        auto uid                     = constraint->uid();
        uid_to_constraint_index[uid] = i;
        constraint->m_index          = i;
    }

    constraint_geo_info_offsets_counts.resize(constraints.view().size());
    span<IndexT> constraint_geo_info_counts = constraint_geo_info_offsets_counts.counts();

    auto  geo_slots       = world.scene().geometries();
    auto& inter_geo_infos = manager->m_impl.inter_geo_infos;
    vector<list<AnimatedInterGeoInfo>> anim_geo_info_buffer;
    anim_geo_info_buffer.resize(constraint_view.size());

    // filter-out the geo infos for each constraint
    for(auto& info : inter_geo_infos)
    {
        auto  geo_slot = geo_slots[info.geo_slot_index];
        auto& geo      = geo_slot->geometry();
        auto  uids     = geo.meta().find<VectorXu64>(builtin::constraint_uids);
        if(uids)
        {
            auto uid_values = uids->view().front();
            for(auto uid_value : uid_values)
            {
                auto it = uid_to_constraint_index.find(uid_value);
                UIPC_ASSERT(it != uid_to_constraint_index.end(),
                            "InterAffineBodyAnimator: No responsible backend SimSystem registered for constraint uid {}",
                            uid_value);
                auto index = it->second;
                anim_geo_info_buffer[index].push_back(info);
            }
        }
    }

    // count the geo infos for each constraint
    std::ranges::transform(anim_geo_info_buffer,
                           constraint_geo_info_counts.begin(),
                           [](const list<AnimatedInterGeoInfo>& infos)
                           { return static_cast<IndexT>(infos.size()); });

    // get offset
    constraint_geo_info_offsets_counts.scan();

    auto total_count = constraint_geo_info_offsets_counts.total_count();
    anim_geo_infos.reserve(total_count);

    // copy the geo infos to the flatten buffer
    for(auto&& infos : anim_geo_info_buffer)
    {
        for(auto& info : infos)
        {
            anim_geo_infos.push_back(info);
        }
    }

    // initialize the constraints
    for(auto constraint : constraints.view())
    {
        FilteredInfo info{this, constraint->m_index};
        constraint->init(info);
    }

    constraint_energy_offsets_counts.resize(constraint_view.size());
    constraint_gradient_offsets_counts.resize(constraint_view.size());
    constraint_hessian_offsets_counts.resize(constraint_view.size());
}

void InterAffineBodyAnimator::Impl::step()
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

void InterAffineBodyAnimator::compute_energy(ABDLineSearchReporter::EnergyInfo& info)
{
    for(auto constraint : m_impl.constraints.view())
    {
        EnergyInfo this_info{&m_impl, constraint->m_index, m_impl.dt, info.energies()};
        constraint->compute_energy(this_info);
    }
}

void InterAffineBodyAnimator::compute_gradient_hessian(ABDLinearSubsystem::AssembleInfo& info)
{
    for(auto constraint : m_impl.constraints.view())
    {
        GradientHessianInfo this_info{
            &m_impl, constraint->m_index, m_impl.dt, info.gradients(), info.hessians()};
        constraint->compute_gradient_hessian(this_info);
    }
}

span<const InterAffineBodyAnimator::AnimatedInterGeoInfo> InterAffineBodyAnimator::FilteredInfo::anim_inter_geo_infos() const noexcept
{
    auto [offset, count] = m_impl->constraint_geo_info_offsets_counts[m_index];

    return span<const AnimatedInterGeoInfo>{m_impl->anim_geo_infos}.subspan(offset, count);
}

Float InterAffineBodyAnimator::BaseInfo::substep_ratio() const noexcept
{
    return m_impl->global_animator->substep_ratio();
}

muda::CBufferView<Vector12> InterAffineBodyAnimator::BaseInfo::qs() const noexcept
{
    return m_impl->affine_body_dynamics->qs();
}

muda::CBufferView<Vector12> InterAffineBodyAnimator::BaseInfo::q_prevs() const noexcept
{
    return m_impl->affine_body_dynamics->q_prevs();
}

muda::CBufferView<ABDJacobiDyadicMass> InterAffineBodyAnimator::BaseInfo::body_masses() const noexcept
{
    return m_impl->affine_body_dynamics->body_masses();
}

muda::CBufferView<IndexT> InterAffineBodyAnimator::BaseInfo::is_fixed() const noexcept
{
    return m_impl->affine_body_dynamics->body_is_fixed();
}

muda::BufferView<Float> InterAffineBodyAnimator::EnergyInfo::energies() const noexcept
{
    auto [offset, count] = m_impl->constraint_energy_offsets_counts[m_index];
    return m_energies.subview(offset, count);
}

muda::DoubletVectorView<Float, 12> InterAffineBodyAnimator::GradientHessianInfo::gradients() const noexcept
{
    auto [offset, count] = m_impl->constraint_gradient_offsets_counts[m_index];
    return m_gradients.subview(offset, count);
}

muda::TripletMatrixView<Float, 12> InterAffineBodyAnimator::GradientHessianInfo::hessians() const noexcept
{
    auto [offset, count] = m_impl->constraint_hessian_offsets_counts[m_index];
    return m_hessians.subview(offset, count);
}

void InterAffineBodyAnimator::ReportExtentInfo::hessian_block_count(SizeT count) noexcept
{
    m_hessian_block_count = count;
}

void InterAffineBodyAnimator::ReportExtentInfo::gradient_segment_count(SizeT count) noexcept
{
    m_gradient_segment_count = count;
}

void InterAffineBodyAnimator::ReportExtentInfo::energy_count(SizeT count) noexcept
{
    m_energy_count = count;
}
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
class InterAffineBodyAnimatorLinearSubsystemReporter final : public ABDLinearSubsystemReporter
{
  public:
    using ABDLinearSubsystemReporter::ABDLinearSubsystemReporter;
    SimSystemSlot<InterAffineBodyAnimator> animator;

    virtual void do_build(BuildInfo& info) override
    {
        animator = require<InterAffineBodyAnimator>();
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_report_extent(ReportExtentInfo& info) override
    {
        SizeT gradient_count = 0;
        SizeT hessian_count  = 0;

        gradient_count =
            animator->m_impl.constraint_gradient_offsets_counts.total_count();
        hessian_count = animator->m_impl.constraint_hessian_offsets_counts.total_count();

        info.gradient_count(gradient_count);
        info.hessian_count(hessian_count);
    }

    virtual void do_assemble(AssembleInfo& info) override
    {
        animator->compute_gradient_hessian(info);
    }
};

REGISTER_SIM_SYSTEM(InterAffineBodyAnimatorLinearSubsystemReporter);

class InterAffineBodyAnimatorLineSearchSubreporter final : public ABDLineSearchSubreporter
{
  public:
    using ABDLineSearchSubreporter::ABDLineSearchSubreporter;
    SimSystemSlot<InterAffineBodyAnimator> animator;

    virtual void do_build(BuildInfo& info) override
    {
        animator = require<InterAffineBodyAnimator>();
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_report_extent(ExtentInfo& info) override
    {
        SizeT energy_count =
            animator->m_impl.constraint_energy_offsets_counts.total_count();
        info.energy_count(energy_count);
    }

    virtual void do_report_energy(EnergyInfo& info) override
    {
        animator->compute_energy(info);
    }
};

REGISTER_SIM_SYSTEM(InterAffineBodyAnimatorLineSearchSubreporter);
}  // namespace uipc::backend::cuda