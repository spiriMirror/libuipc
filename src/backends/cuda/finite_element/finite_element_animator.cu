#include <finite_element/finite_element_animator.h>
#include <finite_element/finite_element_constraint.h>
#include <uipc/builtin/attribute_name.h>
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
    m_impl.constraints.register_subsystem(*constraint);
}

void FiniteElementAnimator::assemble(AssembleInfo& info)
{
    // compute the gradient and hessian
    for(auto constraint : m_impl.constraints.view())
    {
        ComputeGradientHessianInfo this_info{
            &m_impl, constraint->m_index, info.dt(), info.hessians()};
        constraint->compute_gradient_hessian(this_info);
    }

    // assemble the gradient and hessian
    m_impl.assemble(info);
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

void FiniteElementAnimator::report_extent(ExtentInfo& info)
{
    info.hessian_block_count = m_impl.constraint_hessian_offsets_counts.total_count();
}

void FiniteElementAnimator::Impl::step()
{
    for(auto constraint : constraints.view())
    {
        FilteredInfo info{this, constraint->m_index};
        constraint->step(info);
    }


    // clear the last element
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

    SizeT E_count    = constraint_energy_offsets_counts.total_count();
    SizeT G3_count   = constraint_gradient_offsets_counts.total_count();
    SizeT H3x3_count = constraint_hessian_offsets_counts.total_count();

    // resize the buffers
    IndexT vertex_count = finite_element_method->xs().size();
    constraint_energies.resize(E_count);
    constraint_gradient.resize(vertex_count, G3_count);
    constraint_hessian.resize(vertex_count, vertex_count, H3x3_count);
}

void FiniteElementAnimator::Impl::assemble(AssembleInfo& info)
{
    using namespace muda;

    // only need to setup gradient (from doublet vector to dense vector)
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(constraint_gradient.doublet_count(),
               [anim_gradients = std::as_const(constraint_gradient).viewer().name("aim_gradients"),
                gradients = info.gradients().viewer().name("gradients"),
                is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
               {
                   const auto& [i, G3] = anim_gradients(I);
                   if(is_fixed(i))
                   {
                       //
                   }
                   else
                   {
                       gradients.segment<3>(i * 3).atomic_add(G3);
                   }
               });
}

Float FiniteElementAnimator::compute_energy(LineSearcher::EnergyInfo& info)
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
    return m_impl->constraint_energies.view(offset, count);
}

muda::DoubletVectorView<Float, 3> FiniteElementAnimator::ComputeGradientHessianInfo::gradients() const noexcept
{
    auto [offset, count] = m_impl->constraint_gradient_offsets_counts[m_index];
    return m_impl->constraint_gradient.view().subview(offset, count);
}

void FiniteElementAnimator::ReportExtentInfo::hessian_block_count(SizeT count) noexcept
{
    m_hessian_block_count = count;
}
void FiniteElementAnimator::ReportExtentInfo::gradient_segment_count(SizeT count) noexcept
{
    m_gradient_segment_count = count;
}
void FiniteElementAnimator::ReportExtentInfo::energy_count(SizeT count) noexcept
{
    m_energy_count = count;
}
}  // namespace uipc::backend::cuda
