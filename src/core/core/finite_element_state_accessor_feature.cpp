#include <uipc/core/finite_element_state_accessor_feature.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::core
{
// A default implementation
geometry::SimplicialComplex FiniteElementStateAccessorFeatureOverrider::do_create_geometry(
    IndexT vertex_offset, SizeT vertex_count)
{
    // create an empty simplicial complex
    // we don't want to create default position and transform attributes here
    // user should create them explicitly if needed
    geometry::SimplicialComplex::CreateInfo info;
    info.create_position  = false;
    info.create_transform = false;
    geometry::SimplicialComplex R{info};

    // 1) resize the vertex attribute
    R.vertices().resize(vertex_count);

    // 2) set the vertex offset attribute
    auto vertex_offset_attr =
        R.meta().create<IndexT>(builtin::backend_fem_vertex_offset, 0);
    view(*vertex_offset_attr)[0] = vertex_offset;

    return R;
}

FiniteElementStateAccessorFeature::FiniteElementStateAccessorFeature(
    S<FiniteElementStateAccessorFeatureOverrider> overrider)
    : m_impl(std::move(overrider))
{
    UIPC_ASSERT(m_impl, "FiniteElementStateAccessorFeatureOverrider must not be null.");
}

SizeT FiniteElementStateAccessorFeature::vertex_count() const
{
    return m_impl->get_vertex_count();
}

geometry::SimplicialComplex FiniteElementStateAccessorFeature::create_geometry(IndexT vertex_offset,
                                                                               SizeT vertex_count) const
{
    auto total_vert_num = this->vertex_count();
    UIPC_ASSERT(vertex_offset <= total_vert_num,
                "vertex_offset ({}) must not be larger than total vertex number ({})",
                vertex_offset,
                total_vert_num);

    if(vertex_count == ~0ull)
        vertex_count = total_vert_num - vertex_offset;

    return m_impl->do_create_geometry(vertex_offset, vertex_count);
}

void FiniteElementStateAccessorFeature::copy_from(const geometry::SimplicialComplex& state_geo) const
{
    m_impl->do_copy_from(state_geo);
}

void FiniteElementStateAccessorFeature::copy_to(geometry::SimplicialComplex& state_geo) const
{
    m_impl->do_copy_to(state_geo);
}

std::string_view FiniteElementStateAccessorFeature::get_name() const
{
    return FeatureName;
}
}  // namespace uipc::core
