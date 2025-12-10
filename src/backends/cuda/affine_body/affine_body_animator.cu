#include <affine_body/affine_body_animator.h>
#include <affine_body/affine_body_constraint.h>
#include <uipc/builtin/attribute_name.h>
#include <muda/cub/device/device_reduce.h>
#include <affine_body/abd_line_search_reporter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyAnimator);

void AffineBodyAnimator::do_build(BuildInfo& info)
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
    m_impl.global_animator      = &require<GlobalAnimator>();
    auto dt_attr                = world().scene().config().find<Float>("dt");
    m_impl.dt                   = dt_attr->view()[0];
}

void AffineBodyAnimator::add_constraint(AffineBodyConstraint* constraint)
{
    m_impl.constraints.register_subsystem(*constraint);
}

void AffineBodyAnimator::do_init()
{
    m_impl.init(world());
}

void AffineBodyAnimator::do_step()
{
    m_impl.step();
}

void AffineBodyAnimator::Impl::init(backend::WorldVisitor& world)
{
    // sort the constraints by uid
    auto constraint_view = constraints.view();

    std::sort(constraint_view.begin(),
              constraint_view.end(),
              [](const AffineBodyConstraint* a, const AffineBodyConstraint* b)
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

    auto  geo_slots = world.scene().geometries();
    auto& geo_infos = affine_body_dynamics->m_impl.geo_infos;
    vector<list<AnimatedGeoInfo>> anim_geo_info_buffer;
    anim_geo_info_buffer.resize(constraint_view.size());

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
                            "AffineBodyAnimator: No responsible backend SimSystem registered for constraint uid {}",
                            uid_value);
                auto index = it->second;
                anim_geo_info_buffer[index].push_back(info);
            }
        }
    }

    // fill in the counts
    std::ranges::transform(anim_geo_info_buffer,
                           constraint_geo_info_counts.begin(),
                           [](const auto& infos)
                           { return static_cast<IndexT>(infos.size()); });

    // scan to get offsets and total count
    constraint_geo_info_offsets_counts.scan();

    auto total_anim_geo_info_count = constraint_geo_info_offsets_counts.total_count();
    anim_geo_infos.reserve(total_anim_geo_info_count);

    for(auto& infos : anim_geo_info_buffer)
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

void AffineBodyAnimator::Impl::step()
{
    // Step constraints
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

void AffineBodyAnimator::compute_energy(ABDLineSearchReporter::EnergyInfo& info)
{
    for(auto constraint : m_impl.constraints.view())
    {
        EnergyInfo this_info{&m_impl, constraint->m_index, m_impl.dt, info.energies()};
        constraint->compute_energy(this_info);
    }
}

void AffineBodyAnimator::compute_gradient_hessian(ABDLinearSubsystem::AssembleInfo& info)
{
    for(auto constraint : m_impl.constraints.view())
    {
        GradientHessianInfo this_info{
            &m_impl, constraint->m_index, m_impl.dt, info.gradients(), info.hessians()};
        constraint->compute_gradient_hessian(this_info);
    }
}

auto AffineBodyAnimator::FilteredInfo::anim_geo_infos() const noexcept
    -> span<const AnimatedGeoInfo>
{
    auto [offset, count] = m_impl->constraint_geo_info_offsets_counts[m_index];

    return span<const AnimatedGeoInfo>{m_impl->anim_geo_infos}.subspan(offset, count);
}

Float AffineBodyAnimator::BaseInfo::substep_ratio() const noexcept
{
    return m_impl->global_animator->substep_ratio();
}

muda::CBufferView<Vector12> AffineBodyAnimator::BaseInfo::qs() const noexcept
{
    return m_impl->affine_body_dynamics->m_impl.body_id_to_q.view();
}

muda::CBufferView<Vector12> AffineBodyAnimator::BaseInfo::q_prevs() const noexcept
{
    return m_impl->affine_body_dynamics->m_impl.body_id_to_q_prev.view();
}

muda::CBufferView<ABDJacobiDyadicMass> AffineBodyAnimator::BaseInfo::body_masses() const noexcept
{
    return m_impl->affine_body_dynamics->m_impl.body_id_to_abd_mass.view();
}

muda::CBufferView<IndexT> AffineBodyAnimator::BaseInfo::is_fixed() const noexcept
{
    return m_impl->affine_body_dynamics->m_impl.body_id_to_is_fixed.view();
}

muda::BufferView<Float> AffineBodyAnimator::EnergyInfo::energies() const noexcept
{
    auto [offset, count] = m_impl->constraint_energy_offsets_counts[m_index];
    return m_energies.subview(offset, count);
}

muda::DoubletVectorView<Float, 12> AffineBodyAnimator::GradientHessianInfo::gradients() const noexcept
{
    auto [offset, count] = m_impl->constraint_gradient_offsets_counts[m_index];
    return m_gradients.subview(offset, count);
}

muda::TripletMatrixView<Float, 12> AffineBodyAnimator::GradientHessianInfo::hessians() const noexcept
{
    auto [offset, count] = m_impl->constraint_hessian_offsets_counts[m_index];
    return m_hessians.subview(offset, count);
}

void AffineBodyAnimator::ReportExtentInfo::hessian_block_count(SizeT count) noexcept
{
    m_hessian_block_count = count;
}

void AffineBodyAnimator::ReportExtentInfo::gradient_segment_count(SizeT count) noexcept
{
    m_gradient_segment_count = count;
}

void AffineBodyAnimator::ReportExtentInfo::energy_count(SizeT count) noexcept
{
    m_energy_count = count;
}
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
class AffineBodyAnimatorLinearSubsystemReporter final : public ABDLinearSubsystemReporter
{
  public:
    using ABDLinearSubsystemReporter::ABDLinearSubsystemReporter;
    SimSystemSlot<AffineBodyAnimator> animator;

    virtual void do_build(BuildInfo& info) override
    {
        animator = require<AffineBodyAnimator>();
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

REGISTER_SIM_SYSTEM(AffineBodyAnimatorLinearSubsystemReporter);

class AffineBodyAnimatorLineSearchSubreporter final : public ABDLineSearchSubreporter
{
  public:
    using ABDLineSearchSubreporter::ABDLineSearchSubreporter;
    SimSystemSlot<AffineBodyAnimator> animator;

    virtual void do_build(BuildInfo& info) override
    {
        animator = require<AffineBodyAnimator>();
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

REGISTER_SIM_SYSTEM(AffineBodyAnimatorLineSearchSubreporter);
}  // namespace uipc::backend::cuda