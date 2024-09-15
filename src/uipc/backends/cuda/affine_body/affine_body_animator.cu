#include <affine_body/affine_body_animator.h>
#include <affine_body/affine_body_constraint.h>
#include <uipc/builtin/attribute_name.h>
#include <muda/cub/device/device_reduce.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyAnimator);

void AffineBodyAnimator::do_build(BuildInfo& info)
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();

    m_impl.dt = world().scene().info()["dt"].get<Float>();
}

void AffineBodyAnimator::add_constraint(AffineBodyConstraint* constraint)
{
    m_impl.constraints.register_subsystem(*constraint);
}

void AffineBodyAnimator::assemble(AssembleInfo& info)
{
    for(auto constraint : m_impl.constraints.view())
    {
        ComputeGradientHessianInfo this_info{
            &m_impl, constraint->m_index, m_impl.dt};
        constraint->compute_gradient_hessian(this_info);
    }

    info.gradients = m_impl.constraint_gradient.view();
    info.hessians  = m_impl.constraint_hessian.view();
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

    // TODO:
}

void AffineBodyAnimator::report_extent(ExtentInfo& info)
{
    info.hessian_block_count = m_impl.constraint_hessian_offsets.back();
}

void AffineBodyAnimator::Impl::step()
{
    for(auto constraint : constraints.view())
    {
        FilteredInfo info{this, constraint->m_index};
        constraint->step(info);
    }

    SizeT H12x12_count = 0;
    SizeT G12_count    = 0;
    SizeT E_count      = 0;

    // clear the last element
    constraint_energy_counts.back()   = 0;
    constraint_gradient_counts.back() = 0;
    constraint_hessian_counts.back()  = 0;

    for(auto&& [i, constraint] : enumerate(constraints.view()))
    {
        ReportExtentInfo this_info;
        constraint->report_extent(this_info);

        constraint_energy_counts[i]   = this_info.m_energy_count;
        constraint_gradient_counts[i] = this_info.m_gradient_segment_count;
        constraint_hessian_counts[i]  = this_info.m_hessian_block_count;
    }

    // update the offsets
    std::exclusive_scan(constraint_energy_counts.begin(),
                        constraint_energy_counts.end(),
                        constraint_energy_offsets.begin(),
                        0);

    E_count = constraint_energy_offsets.back();

    std::exclusive_scan(constraint_gradient_counts.begin(),
                        constraint_gradient_counts.end(),
                        constraint_gradient_offsets.begin(),
                        0);

    G12_count = constraint_gradient_offsets.back();

    std::exclusive_scan(constraint_hessian_counts.begin(),
                        constraint_hessian_counts.end(),
                        constraint_hessian_offsets.begin(),
                        0);

    H12x12_count = constraint_hessian_offsets.back();

    // resize the buffers
    IndexT body_count = affine_body_dynamics->m_impl.body_count();
    constraint_energies.resize(E_count);
    constraint_gradient.resize(body_count, G12_count);
    constraint_hessian.resize(body_count, body_count, H12x12_count);
}

Float AffineBodyAnimator::compute_energy(LineSearcher::EnergyInfo& info)
{
    using namespace muda;
    for(auto constraint : m_impl.constraints.view())
    {
        ComputeEnergyInfo this_info{&m_impl, constraint->m_index, info.dt()};
        constraint->compute_energy(this_info);
    }

    DeviceReduce().Sum(m_impl.constraint_energies.data(),
                       m_impl.constraint_energy.data(),
                       m_impl.constraint_energies.size());

    // copy back to host
    Float E = m_impl.constraint_energy;

    return E;
}

void AffineBodyAnimator::compute_gradient_hessian(GradientHessianComputer::ComputeInfo& info)
{
    for(auto constraint : m_impl.constraints.view())
    {
        ComputeGradientHessianInfo this_info{&m_impl, constraint->m_index, info.dt()};
        constraint->compute_gradient_hessian(this_info);
    }
}

auto AffineBodyAnimator::FilteredInfo::animated_geo_infos() const -> span<const AnimatedGeoInfo>
{
    return span<const AnimatedGeoInfo>{m_impl->anim_geo_infos}.subspan(
        m_impl->constraint_geo_info_offsets[m_index],
        m_impl->constraint_geo_info_counts[m_index]);
}

SizeT AffineBodyAnimator::FilteredInfo::anim_body_count() const noexcept
{
    return m_impl->constraint_body_counts[m_index];
}

span<const IndexT> AffineBodyAnimator::FilteredInfo::anim_indices() const
{
    auto offset = m_impl->constraint_body_offsets[m_index];
    auto count  = m_impl->constraint_body_counts[m_index];
    return span{m_impl->anim_indices}.subspan(offset, count);
}

muda::CBufferView<Vector3> AffineBodyAnimator::BaseInfo::xs() const noexcept
{
    return m_impl->finite_element_method->xs();
}

muda::CBufferView<Float> AffineBodyAnimator::BaseInfo::masses() const noexcept
{
    return m_impl->finite_element_method->masses();
}

muda::CBufferView<IndexT> AffineBodyAnimator::BaseInfo::is_fixed() const noexcept
{
    return m_impl->finite_element_method->is_fixed();
}

muda::BufferView<Float> AffineBodyAnimator::ComputeEnergyInfo::energies() const noexcept
{
    auto offset = m_impl->constraint_energy_offsets[m_index];
    auto count  = m_impl->constraint_energy_counts[m_index];
    return m_impl->constraint_energies.view(offset, count);
}

muda::DoubletVectorView<Float, 12> AffineBodyAnimator::ComputeGradientHessianInfo::gradients() const noexcept
{
    auto offset = m_impl->constraint_gradient_offsets[m_index];
    auto count  = m_impl->constraint_gradient_counts[m_index];
    return m_impl->constraint_gradient.view().subview(offset, count);
}

muda::TripletMatrixView<Float, 12> AffineBodyAnimator::ComputeGradientHessianInfo::hessians() const noexcept
{
    auto offset = m_impl->constraint_hessian_offsets[m_index];
    auto count  = m_impl->constraint_hessian_counts[m_index];
    return m_impl->constraint_hessian.view().subview(offset, count);
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
