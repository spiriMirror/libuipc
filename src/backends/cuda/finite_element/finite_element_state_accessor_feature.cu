#include <finite_element/finite_element_state_accessor_feature.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_vertex_reporter.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
FiniteElementStateAccessorFeatureOverrider::FiniteElementStateAccessorFeatureOverrider(
    FiniteElementMethod& fem, FiniteElementVertexReporter& vertex_reporter)
    : m_fem{fem}
    , m_vertex_reporter{vertex_reporter}
{
}

SizeT FiniteElementStateAccessorFeatureOverrider::get_vertex_count()
{
    return m_fem.xs().size();
}

void FiniteElementStateAccessorFeatureOverrider::do_copy_from(const geometry::SimplicialComplex& state_geo)
{
    auto v_offset_attr = state_geo.meta().find<IndexT>(builtin::backend_fem_vertex_offset);
    UIPC_ASSERT(v_offset_attr, "Cannot find `backend_fem_vertex_offset` on State Geometry, why can it happen?");
    auto v_offset = v_offset_attr->view()[0];
    auto v_count  = state_geo.vertices().size();


    // 1. Position
    auto pos = state_geo.vertices().find<Vector3>(builtin::position);
    if(pos)
    {
        auto pos_view  = pos->view();
        auto x_subview = m_fem.m_impl.xs.view(v_offset, v_count);
        x_subview.copy_from(pos_view.data());

        // Sync positions to slot geometries so consumers reading
        // geo->positions() see the updated state without waiting for
        // retrieve()/write_scene().
        auto geo_slots = m_fem.world().scene().geometries();
        for(auto&& [i, info] : enumerate(m_fem.m_impl.geo_infos))
        {
            if(info.vertex_offset + info.vertex_count <= v_offset
               || info.vertex_offset >= v_offset + v_count)
                continue;

            auto& geo_slot = geo_slots[info.geo_slot_index];
            auto* sc       = geo_slot->geometry().as<geometry::SimplicialComplex>();
            if(!sc)
                continue;
            auto dst = geometry::view(sc->positions());
            auto src_offset = std::max(info.vertex_offset, static_cast<SizeT>(v_offset));
            auto src_end    = std::min(info.vertex_offset + info.vertex_count, static_cast<SizeT>(v_offset) + v_count);
            auto dst_start  = src_offset - info.vertex_offset;
            auto pos_start  = src_offset - static_cast<SizeT>(v_offset);
            for(SizeT k = 0; k < src_end - src_offset; ++k)
                dst[dst_start + k] = pos_view[pos_start + k];
        }
    }

    // 2. Velocity
    auto vel = state_geo.vertices().find<Vector3>(builtin::velocity);
    if(vel)
    {
        auto vel_view  = vel->view();
        auto v_subview = m_fem.m_impl.vs.view(v_offset, v_count);
        v_subview.copy_from(vel_view.data());
    }

    // request the vertex reporter to update attributes
    m_vertex_reporter.request_attribute_update();
}

void FiniteElementStateAccessorFeatureOverrider::do_copy_to(geometry::SimplicialComplex& state_geo)
{
    auto v_offset_attr = state_geo.meta().find<IndexT>(builtin::backend_fem_vertex_offset);
    UIPC_ASSERT(v_offset_attr, "Cannot find `backend_fem_vertex_offset` on State Geometry, why can it happen?");
    auto v_offset = v_offset_attr->view()[0];
    auto v_count  = state_geo.vertices().size();

    // 1. Position
    auto pos = state_geo.vertices().find<Vector3>(builtin::position);
    if(pos)
    {
        auto pos_view  = view(*pos);
        auto x_subview = m_fem.m_impl.xs.view(v_offset, v_count);
        x_subview.copy_to(pos_view.data());
    }

    // 2. Velocity
    auto vel = state_geo.vertices().find<Vector3>(builtin::velocity);
    if(vel)
    {
        auto vel_view  = view(*vel);
        auto v_subview = m_fem.m_impl.vs.view(v_offset, v_count);
        v_subview.copy_to(vel_view.data());
    }
}
}  // namespace uipc::backend::cuda
