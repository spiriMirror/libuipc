#include <finite_element/fem_exporter_manager.h>
#include <finite_element/fem_system_feature.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_elastics.h>
#include <finite_element/finite_element_constitution.h>
#include <finite_element/finite_element_extra_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <fmt/ranges.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void FEMExporterManager_aggregate_vertex_gradient_kernel(
        cuda_tool::CDoubletVectorView<Float, 3> gradients,
        cuda_tool::BufferView<Vector3>          dst,
        int                                     n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;

        auto&& [i, g] = gradients(I);
        Vector3& v    = dst(i);
        atomicAdd(v.data() + 0, g[0]);
        atomicAdd(v.data() + 1, g[1]);
        atomicAdd(v.data() + 2, g[2]);
    }

    // find-or-create an instances attribute
    template <typename T>
    static S<geometry::AttributeSlot<T>> instance_attribute(geometry::Geometry& geo,
                                                            std::string_view name,
                                                            const T& default_value)
    {
        auto attr = geo.instances().find<T>(name);
        if(!attr)
            attr = geo.instances().create<T>(name, default_value);
        return attr;
    }
}  // namespace

REGISTER_SIM_SYSTEM(FEMExporterManager);

void FEMExporterManager::do_build()
{
    m_fem      = require<FiniteElementMethod>();
    m_elastics = require<FiniteElementElastics>();
    m_dt_attr  = world().scene().config().find<Float>("dt");
    UIPC_ASSERT(m_dt_attr, "Scene config must have a 'dt' attribute.");

    auto overrider = std::make_shared<FEMSystemFeatureOverrider>(this);
    auto feature   = std::make_shared<core::FEMSystemFeature>(overrider);
    features().insert(feature);
}

FEMExporterManager::ConstitutionLookup FEMExporterManager::find_constitution(U64 uid) const
{
    auto& fem_impl = m_fem->m_impl;

    auto constitutions = fem_impl.constitutions.view();
    auto extras        = fem_impl.extra_constitutions.view();

    ConstitutionLookup lookup;
    lookup.elastics_index = ~0ull;

    for(SizeT I = 0; I < constitutions.size(); ++I)
    {
        auto* c = constitutions[I];
        if(c->uid() == uid)
        {
            lookup.base           = c;
            lookup.dim            = c->dim();
            lookup.elastics_index = I;
            break;
        }
    }

    if(!lookup.base)
    {
        for(SizeT I = 0; I < extras.size(); ++I)
        {
            if(extras[I]->uid() == uid)
            {
                lookup.extra          = extras[I];
                lookup.elastics_index = constitutions.size() + I;
                break;
            }
        }
    }

    if(!lookup.base && !lookup.extra)
    {
        logger::warn(R"(FEMExporterManager: constitution uid '{}' not found
Supported uids are: [{}])",
                     uid,
                     fmt::join(get_fem_constitution_uids(), ", "));
    }
    else if(lookup.base)
    {
        // per-dim classification gives the primitive range (exported topology)
        const FiniteElementMethod::ConstitutionInfo* info = nullptr;
        switch(lookup.dim)
        {
            case 0: {
                auto it = fem_impl.codim_0d_uid_to_index.find(uid);
                UIPC_ASSERT(it != fem_impl.codim_0d_uid_to_index.end(),
                            "Constitution uid {} missing from the 0D classification",
                            uid);
                info = &fem_impl.codim_0d_constitution_infos[it->second];
                break;
            }
            case 1: {
                auto it = fem_impl.codim_1d_uid_to_index.find(uid);
                UIPC_ASSERT(it != fem_impl.codim_1d_uid_to_index.end(),
                            "Constitution uid {} missing from the 1D classification",
                            uid);
                info = &fem_impl.codim_1d_constitution_infos[it->second];
                break;
            }
            case 2: {
                auto it = fem_impl.codim_2d_uid_to_index.find(uid);
                UIPC_ASSERT(it != fem_impl.codim_2d_uid_to_index.end(),
                            "Constitution uid {} missing from the 2D classification",
                            uid);
                info = &fem_impl.codim_2d_constitution_infos[it->second];
                break;
            }
            case 3: {
                auto it = fem_impl.fem_3d_uid_to_index.find(uid);
                UIPC_ASSERT(it != fem_impl.fem_3d_uid_to_index.end(),
                            "Constitution uid {} missing from the 3D classification",
                            uid);
                info = &fem_impl.fem_3d_constitution_infos[it->second];
                break;
            }
            default:
                UIPC_ASSERT(
                    false, "Invalid dim {} for constitution uid {}", lookup.dim, uid);
                break;
        }

        if(info)
        {
            lookup.primitive_offset = info->primitive_offset;
            lookup.primitive_count  = info->primitive_count;
        }
    }

    return lookup;
}

vector<U64> FEMExporterManager::get_fem_constitution_uids() const
{
    auto& fem_impl = m_fem->m_impl;

    vector<U64> uids;
    for(auto&& c : fem_impl.constitutions.view())
        uids.push_back(c->uid());
    for(auto&& c : fem_impl.extra_constitutions.view())
        uids.push_back(c->uid());
    return uids;
}

void FEMExporterManager::get_fem_energy(U64 uid, geometry::Geometry& prim_energy)
{
    auto lookup = find_constitution(uid);
    if(lookup.elastics_index == ~0ull)
        return;

    auto& el = m_elastics->m_impl;

    auto [offset, count] = el.constitution_energy_offsets_counts[lookup.elastics_index];
    auto total = el.constitution_energy_offsets_counts.total_count();

    // regenerated by the compute below (discard growth)
    m_energies.resize_discard(total);

    // the dt keeps the solver's dt^2 incremental-potential scaling
    FiniteElementElastics::ComputeEnergyInfo info{
        &el, lookup.elastics_index, m_dt_attr->view()[0], m_energies.view()};

    if(lookup.base)
        lookup.base->compute_energy(info);
    else
        lookup.extra->compute_energy(info);

    auto uid_attr = prim_energy.meta().find<U64>("uid");
    if(!uid_attr)
        uid_attr = prim_energy.meta().create<U64>("uid");
    view(*uid_attr)[0] = uid;

    prim_energy.instances().resize(count);
    auto energy_attr = instance_attribute<Float>(prim_energy, "energy", 0.0);
    m_energies.view(offset, count).copy_to(view(*energy_attr).data());

    // element topology, only defined for the base (non-extra) constitutions
    if(lookup.base)
    {
        auto& fem_impl = m_fem->m_impl;
        switch(lookup.dim)
        {
            case 3: {
                auto topo =
                    instance_attribute<Vector4i>(prim_energy, "topo", Vector4i::Zero());
                fem_impl.tets
                    .view(lookup.primitive_offset, lookup.primitive_count)
                    .copy_to(view(*topo).data());
                break;
            }
            case 2: {
                auto topo =
                    instance_attribute<Vector3i>(prim_energy, "topo", Vector3i::Zero());
                fem_impl.codim_2ds
                    .view(lookup.primitive_offset, lookup.primitive_count)
                    .copy_to(view(*topo).data());
                break;
            }
            case 1: {
                auto topo =
                    instance_attribute<Vector2i>(prim_energy, "topo", Vector2i::Zero());
                fem_impl.codim_1ds
                    .view(lookup.primitive_offset, lookup.primitive_count)
                    .copy_to(view(*topo).data());
                break;
            }
            default:
                break;  // 0D (particles): one energy per particle, no element topology
        }
    }
}

void FEMExporterManager::get_fem_gradient(U64 uid, geometry::Geometry& prim_grad)
{
    auto lookup = find_constitution(uid);
    if(lookup.elastics_index == ~0ull)
        return;

    auto& el = m_elastics->m_impl;

    auto [offset, count] = el.constitution_gradient_offsets_counts[lookup.elastics_index];
    auto total        = el.constitution_gradient_offsets_counts.total_count();
    auto vertex_count = m_fem->m_impl.xs.size();

    // regenerated by the compute below (discard growth)
    m_gradients.reshape((int)vertex_count);
    m_gradients.resize_doublets_discard(total);

    FiniteElementElastics::ComputeGradientHessianInfo info{&el,
                                                           lookup.elastics_index,
                                                           /*gradient_only=*/true,
                                                           /*dt=*/m_dt_attr->view()[0],
                                                           m_gradients.view(),
                                                           {}};

    if(lookup.base)
        lookup.base->compute_gradient_hessian(info);
    else
        lookup.extra->compute_gradient_hessian(info);

    auto uid_attr = prim_grad.meta().find<U64>("uid");
    if(!uid_attr)
        uid_attr = prim_grad.meta().create<U64>("uid");
    view(*uid_attr)[0] = uid;

    prim_grad.instances().resize(count);

    auto slice = m_gradients.view().subview((int)offset, (int)count);

    auto i_attr = instance_attribute<IndexT>(prim_grad, "i", -1);
    slice.indices().copy_to(view(*i_attr).data());

    auto grad_attr = instance_attribute<Vector3>(prim_grad, "grad", Vector3::Zero());
    slice.values().copy_to(view(*grad_attr).data());
}

void FEMExporterManager::get_fem_hessian(U64 uid, geometry::Geometry& prim_hess)
{
    auto lookup = find_constitution(uid);
    if(lookup.elastics_index == ~0ull)
        return;

    auto& el = m_elastics->m_impl;

    auto [offset, count] = el.constitution_hessian_offsets_counts[lookup.elastics_index];
    auto total = el.constitution_hessian_offsets_counts.total_count();
    auto total_gradients = el.constitution_gradient_offsets_counts.total_count();
    auto vertex_count = m_fem->m_impl.xs.size();

    // regenerated by the compute below (discard growth);
    // gradient_only=false also writes the gradient doublets
    m_gradients.reshape((int)vertex_count);
    m_gradients.resize_doublets_discard(total_gradients);
    m_hessians.reshape((int)vertex_count, (int)vertex_count);
    m_hessians.resize_triplets_discard(total);

    FiniteElementElastics::ComputeGradientHessianInfo info{&el,
                                                           lookup.elastics_index,
                                                           /*gradient_only=*/false,
                                                           /*dt=*/m_dt_attr->view()[0],
                                                           m_gradients.view(),
                                                           m_hessians.view()};

    if(lookup.base)
        lookup.base->compute_gradient_hessian(info);
    else
        lookup.extra->compute_gradient_hessian(info);

    auto uid_attr = prim_hess.meta().find<U64>("uid");
    if(!uid_attr)
        uid_attr = prim_hess.meta().create<U64>("uid");
    view(*uid_attr)[0] = uid;

    prim_hess.instances().resize(count);

    auto slice = m_hessians.view().subview((int)offset, (int)count);

    auto i_attr = instance_attribute<IndexT>(prim_hess, "i", -1);
    slice.row_indices().copy_to(view(*i_attr).data());

    auto j_attr = instance_attribute<IndexT>(prim_hess, "j", -1);
    slice.col_indices().copy_to(view(*j_attr).data());

    auto hess_attr = instance_attribute<Matrix3x3>(prim_hess, "hess", Matrix3x3::Zero());
    slice.values().copy_to(view(*hess_attr).data());
}

void FEMExporterManager::get_fem_vertex_gradient(U64 uid, geometry::SimplicialComplex& vert_grad)
{
    auto lookup = find_constitution(uid);
    if(lookup.elastics_index == ~0ull)
        return;

    auto& fem_impl = m_fem->m_impl;

    auto v_offset_attr = vert_grad.meta().find<IndexT>(builtin::backend_fem_vertex_offset);
    UIPC_ASSERT(v_offset_attr,
                "Cannot find `backend_fem_vertex_offset` on the geometry, it must be created by "
                "`FiniteElementStateAccessorFeature::create_geometry()`.");
    auto v_offset = v_offset_attr->view()[0];
    auto v_count  = vert_grad.vertices().size();

    auto& el = m_elastics->m_impl;

    auto [offset, count] = el.constitution_gradient_offsets_counts[lookup.elastics_index];
    auto total        = el.constitution_gradient_offsets_counts.total_count();
    auto vertex_count = fem_impl.xs.size();

    // regenerated by the compute below (discard growth)
    m_gradients.reshape((int)vertex_count);
    m_gradients.resize_doublets_discard(total);

    FiniteElementElastics::ComputeGradientHessianInfo info{&el,
                                                           lookup.elastics_index,
                                                           /*gradient_only=*/true,
                                                           /*dt=*/m_dt_attr->view()[0],
                                                           m_gradients.view(),
                                                           {}};

    if(lookup.base)
        lookup.base->compute_gradient_hessian(info);
    else
        lookup.extra->compute_gradient_hessian(info);

    // aggregate the per-element doublets into a dense per-vertex gradient
    m_vertex_gradients.resize_discard(vertex_count);
    m_vertex_gradients.fill(Vector3::Zero());

    if(count > 0)
    {
        auto k     = FEMExporterManager_aggregate_vertex_gradient_kernel;
        int  n     = (int)count;
        auto slice = m_gradients.view().subview((int)offset, (int)count);
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            slice, m_vertex_gradients.view(), n);
    }

    auto grad_attr = vert_grad.vertices().find<Vector3>("gradient");
    if(!grad_attr)
        grad_attr = vert_grad.vertices().create<Vector3>("gradient", Vector3::Zero());
    m_vertex_gradients.view(v_offset, v_count).copy_to(view(*grad_attr).data());
}
}  // namespace uipc::backend::cuda
