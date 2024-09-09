#include <finite_element/finite_element_animator.h>
#include <finite_element/finite_element_constraint.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/enumerate.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FiniteElementAnimator);

void FiniteElementAnimator::do_build(BuildInfo& info)
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();

    m_impl.dt = world().scene().info()["dt"].get<Float>();
}

void FiniteElementAnimator::add_constraint(FiniteElementConstraint* constraint)
{
    m_impl.constraints.register_subsystem(*constraint);
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

    anim_geo_infos.resize(constraint_view.size());
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
                        "Constraint: Constraint uid not found");
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
            for(int i = 0; i < info.vertex_count; ++i)
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
    h_anim_indices.resize(total_vertex_count);

    // expand the indices
    for(auto&& [i, indices] : enumerate(constraint_vertex_indices))
    {
        auto offset = constraint_vertex_offsets[i];
        for(auto& index : indices)
        {
            h_anim_indices[offset++] = index;
        }
    }

    anim_indices.resize(total_vertex_count);
    anim_indices.view().copy_from(h_anim_indices.data());

    // initialize the constraints
    for(auto constraint : constraint_view)
    {
        FilteredInfo info{this, constraint->m_index};
        constraint->init(info);
    }
}

void FiniteElementAnimator::Impl::step()
{
    for(auto constraint : constraints.view())
    {
        FilteredInfo info{this, constraint->m_index};
        constraint->step(info);
    }
}

void FiniteElementAnimator::compute_energy(LineSearcher::EnergyInfo& info)
{
    for(auto constraint : m_impl.constraints.view())
    {
        ComputeEnergyInfo this_info{&m_impl, constraint->m_index, info.dt()};
        constraint->compute_energy(this_info);
    }
}

void FiniteElementAnimator::compute_gradient_hessian(GradientHessianComputer::ComputeInfo& info)
{
    for(auto constraint : m_impl.constraints.view())
    {
        ComputeGradientHessianInfo this_info{&m_impl, constraint->m_index, info.dt()};
        constraint->compute_gradient_hessian(this_info);
    }
}

auto FiniteElementAnimator::FilteredInfo::animated_geo_infos() const
    -> span<const AnimatedGeoInfo>
{
    return span<const AnimatedGeoInfo>{m_impl->anim_geo_infos}.subspan(
        m_impl->constraint_geo_info_offsets[m_index],
        m_impl->constraint_geo_info_counts[m_index]);
}

span<const IndexT> FiniteElementAnimator::FilteredInfo::anim_indices() const
{
    auto offset = m_impl->constraint_vertex_offsets[m_index];
    auto count  = m_impl->constraint_vertex_counts[m_index];
    return span{m_impl->h_anim_indices}.subspan(offset, count);
}

muda::CBufferView<Vector3> FiniteElementAnimator::BaseInfo::xs() const noexcept
{
    return m_impl->finite_element_method->xs();
}

muda::CBufferView<IndexT> FiniteElementAnimator::BaseInfo::anim_indices() const noexcept
{
    auto offset = m_impl->constraint_vertex_offsets[m_index];
    auto count  = m_impl->constraint_vertex_counts[m_index];
    return m_impl->anim_indices.view(offset, count);
}

muda::CBufferView<Float> FiniteElementAnimator::BaseInfo::masses() const noexcept
{
    return m_impl->finite_element_method->masses();
}

muda::CBufferView<FiniteElementMethod::FixType> FiniteElementAnimator::BaseInfo::is_fixed() const noexcept
{
    return m_impl->finite_element_method->is_fixed();
}

muda::BufferView<Float> FiniteElementAnimator::ComputeEnergyInfo::energies() const noexcept
{
    return m_impl->finite_element_method->m_impl.vertex_kinetic_energies.view();
}

muda::BufferView<Vector3> FiniteElementAnimator::ComputeGradientHessianInfo::gradients() const noexcept
{
    return m_impl->finite_element_method->m_impl.G3s.view();
}

muda::BufferView<Matrix3x3> FiniteElementAnimator::ComputeGradientHessianInfo::hessians() const noexcept
{
    return m_impl->finite_element_method->m_impl.H3x3s.view();
}
}  // namespace uipc::backend::cuda
