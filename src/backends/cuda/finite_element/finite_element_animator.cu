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

    // +1 for total count
    constraint_geo_info_counts.resize(constraint_view.size() + 1, 0);
    constraint_geo_info_offsets.resize(constraint_view.size() + 1, 0);

    auto        geo_slots = world.scene().geometries();
    const auto& geo_infos = finite_element_method->m_impl.geo_infos;

    for(auto& info : geo_infos)
    {
        auto  geo_slot = geo_slots[info.geo_slot_index];
        auto& geo      = geo_slot->geometry();
        auto  uid      = geo.meta().find<U64>(builtin::constraint_uid);
        if(uid)
        {
            auto uid_value = uid->view().front();
            auto it        = uid_to_constraint_index.find(uid_value);
            UIPC_ASSERT(it != uid_to_constraint_index.end(),
                        "FiniteElementAnimator: Constraint uid not found");
            auto index = it->second;
            constraint_geo_info_counts[index]++;
        }
    }

    std::exclusive_scan(constraint_geo_info_counts.begin(),
                        constraint_geo_info_counts.end(),
                        constraint_geo_info_offsets.begin(),
                        0);

    auto total_anim_geo_info = constraint_geo_info_offsets.back();
    anim_geo_infos.resize(total_anim_geo_info);

    vector<SizeT> anim_geo_info_counter(constraint_view.size(), 0);

    for(auto& info : geo_infos)
    {
        auto  geo_slot = geo_slots[info.geo_slot_index];
        auto& geo      = geo_slot->geometry();
        auto  uid      = geo.meta().find<U64>(builtin::constraint_uid);
        if(uid)
        {
            auto uid_value = uid->view().front();
            auto it        = uid_to_constraint_index.find(uid_value);
            UIPC_ASSERT(it != uid_to_constraint_index.end(),
                        "Constraint: Constraint uid not found");
            auto index = it->second;
            auto offset =
                constraint_geo_info_offsets[index] + anim_geo_info_counter[index];
            anim_geo_infos[offset] = info;
            anim_geo_info_counter[index]++;
        }
    }

    vector<list<IndexT>> constraint_vertex_indices(constraint_view.size());
    for(auto& c : constraint_view)
    {
        auto constraint_geo_infos =
            span{anim_geo_infos}.subspan(constraint_geo_info_offsets[c->m_index],
                                         constraint_geo_info_counts[c->m_index]);

        auto& indices = constraint_vertex_indices[c->m_index];

        for(auto& info : constraint_geo_infos)
        {
            for(int i = 0; i < info.vertex_count; i++)
            {
                indices.push_back(info.vertex_offset + i);
            }
        }
    }

    constraint_vertex_counts.resize(constraint_view.size() + 1, 0);
    constraint_vertex_offsets.resize(constraint_view.size() + 1, 0);

    std::ranges::transform(constraint_vertex_indices,
                           constraint_vertex_counts.begin(),
                           [](const auto& indices) { return indices.size(); });

    std::exclusive_scan(constraint_vertex_counts.begin(),
                        constraint_vertex_counts.end(),
                        constraint_vertex_offsets.begin(),
                        0);

    auto total_vertex_count = constraint_vertex_offsets.back();
    anim_indices.resize(total_vertex_count);

    // expand the indices
    for(auto&& [i, indices] : enumerate(constraint_vertex_indices))
    {
        auto offset = constraint_vertex_offsets[i];
        for(auto& index : indices)
        {
            anim_indices[offset++] = index;
        }
    }

    // initialize the constraints
    for(auto constraint : constraint_view)
    {
        FilteredInfo info{this, constraint->m_index};
        constraint->init(info);
    }

    // reserve offsets and counts for constraints (+1 for total count)
    constraint_energy_offsets.resize(constraint_view.size() + 1, 0);
    constraint_energy_counts.resize(constraint_view.size() + 1, 0);
    constraint_gradient_offsets.resize(constraint_view.size() + 1, 0);
    constraint_gradient_counts.resize(constraint_view.size() + 1, 0);
    constraint_hessian_offsets.resize(constraint_view.size() + 1, 0);
    constraint_hessian_counts.resize(constraint_view.size() + 1, 0);
}

void FiniteElementAnimator::report_extent(ExtentInfo& info)
{
    info.hessian_block_count = m_impl.constraint_hessian_offsets.back();
}

void FiniteElementAnimator::Impl::step()
{
    for(auto constraint : constraints.view())
    {
        FilteredInfo info{this, constraint->m_index};
        constraint->step(info);
    }

    SizeT H3x3_count = 0;
    SizeT G3_count   = 0;
    SizeT E_count    = 0;

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

    G3_count = constraint_gradient_offsets.back();

    std::exclusive_scan(constraint_hessian_counts.begin(),
                        constraint_hessian_counts.end(),
                        constraint_hessian_offsets.begin(),
                        0);

    H3x3_count = constraint_hessian_offsets.back();

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
                gradient = info.gradients().viewer().name("gradient"),
                is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
               {
                   const auto& [i, G3] = anim_gradients(I);
                   if(is_fixed(i))
                   {
                       //
                   }
                   else
                   {
                       gradient.segment<3>(i * 3).atomic_add(G3);
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
    return span<const AnimatedGeoInfo>{m_impl->anim_geo_infos}.subspan(
        m_impl->constraint_geo_info_offsets[m_index],
        m_impl->constraint_geo_info_counts[m_index]);
}

SizeT FiniteElementAnimator::FilteredInfo::anim_vertex_count() const noexcept
{
    return m_impl->constraint_vertex_counts[m_index];
}

span<const IndexT> FiniteElementAnimator::FilteredInfo::anim_indices() const
{
    auto offset = m_impl->constraint_vertex_offsets[m_index];
    auto count  = m_impl->constraint_vertex_counts[m_index];
    return span{m_impl->anim_indices}.subspan(offset, count);
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
    auto offset = m_impl->constraint_energy_offsets[m_index];
    auto count  = m_impl->constraint_energy_counts[m_index];
    return m_impl->constraint_energies.view(offset, count);
}

muda::DoubletVectorView<Float, 3> FiniteElementAnimator::ComputeGradientHessianInfo::gradients() const noexcept
{
    auto offset = m_impl->constraint_gradient_offsets[m_index];
    auto count  = m_impl->constraint_gradient_counts[m_index];
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
