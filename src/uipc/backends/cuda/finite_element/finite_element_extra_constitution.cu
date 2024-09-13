#include <finite_element/finite_element_extra_constitution.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
void FiniteElementExtraConstitution::do_build()
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();

    BuildInfo info;
    do_build(info);

    m_impl.finite_element_method;
}

U64 FiniteElementExtraConstitution::uid() const noexcept
{
    return get_uid();
}

span<const FiniteElementMethod::GeoInfo> FiniteElementExtraConstitution::geo_infos() const noexcept
{
    return m_impl.geo_infos;
}

void FiniteElementExtraConstitution::init()
{
    m_impl.init(uid(), world());

    // let the subclass do the rest of the initialization
    FilteredInfo info{&m_impl};
    do_init(info);
}

void FiniteElementExtraConstitution::collect_extent_info()
{
    ReportExtentInfo info;
    do_report_extent(info);
    SizeT N = info.m_energy_count;
    SizeT D = info.m_stencil_dim;

    m_impl.stencil_dim    = D;
    m_impl.energy_count   = N;
    m_impl.gradient_count = N * D;
    m_impl.hessian_count  = N * D * D;
}

void FiniteElementExtraConstitution::compute_energy(FiniteElementMethod::ComputeExtraEnergyInfo& info)
{
    ComputeEnergyInfo this_info{&m_impl, info.dt()};
    do_compute_energy(this_info);
}

void FiniteElementExtraConstitution::compute_gradient_hessian(FiniteElementMethod::ComputeExtraGradientHessianInfo& info)
{
    ComputeGradientHessianInfo this_info{&m_impl, info.dt()};
    do_compute_gradient_hessian(this_info);
}

void FiniteElementExtraConstitution::Impl::init(U64 uid, backend::WorldVisitor& world)
{
    // 1) Find the geometry slots that have the extra constitution uids containing the given uid
    auto& fem_geo_infos = finite_element_method->m_impl.geo_infos;
    auto  geo_slots     = world.scene().geometries();

    list<SizeT> geo_slot_indices;

    SizeT I = 0;

    finite_element_method->for_each(
        geo_slots,
        [](geometry::SimplicialComplex& sc)
        {
            auto uids = sc.meta().find<VectorXu64>(builtin::extra_constitution_uids);
            return uids ? uids->view() : span<const VectorXu64>{};
        },
        [&](SizeT local_i, const VectorXu64& extra_uids)
        {
            UIPC_ASSERT(local_i == 0, "meta data is dim 1, why is local_i not 0?");

            for(auto extra_uid : extra_uids)
            {
                if(extra_uid == uid)
                {
                    geo_slot_indices.push_back(I);
                    break;
                }
            }

            I++;
        });

    geo_infos.resize(geo_slot_indices.size());

    for(auto&& [i, geo_slot_index] : enumerate(geo_slot_indices))
    {
        geo_infos[i] = fem_geo_infos[geo_slot_index];
    }
}

muda::BufferView<Float> FiniteElementExtraConstitution::ComputeEnergyInfo::energies() const noexcept
{
    return m_impl->fem().extra_constitution_energies.view().subview(
        m_impl->energy_offset, m_impl->energy_count);
}

muda::DoubletVectorView<Float, 3> FiniteElementExtraConstitution::ComputeGradientHessianInfo::gradients() const noexcept
{
    return m_impl->fem().extra_constitution_gradient.view().subview(
        m_impl->gradient_offset, m_impl->gradient_count);
}

muda::TripletMatrixView<Float, 3> FiniteElementExtraConstitution::ComputeGradientHessianInfo::hessians() const noexcept
{
    return m_impl->fem().extra_constitution_hessian.view().subview(
        m_impl->hessian_offset, m_impl->hessian_count);
}

void FiniteElementExtraConstitution::ReportExtentInfo::energy_count(SizeT count) noexcept
{
    m_energy_count = count;
}

void FiniteElementExtraConstitution::ReportExtentInfo::stencil_dim(SizeT dim) noexcept
{
    m_stencil_dim = dim;
}
}  // namespace uipc::backend::cuda