#include <uipc/core/affine_body_state_accessor_feature.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::core
{
geometry::SimplicialComplex AffineBodyStateAccessorFeatureOverrider::do_create_geometry(
    IndexT body_offset, SizeT body_count)
{
    // create an empty simplicial complex
    // we don't want to create default position and transform attributes here
    // user should create them explicitly if needed
    geometry::SimplicialComplex::CreateInfo info;
    info.create_position  = false;
    info.create_transform = false;
    geometry::SimplicialComplex R{info};

    // 1) resize the instance attribute
    R.instances().resize(body_count);

    // 2) set the body offset attribute
    auto body_offset_attr = R.meta().create<IndexT>(builtin::backend_abd_body_offset);
    view(*body_offset_attr)[0] = body_offset;

    return R;
}

AffineBodyStateAccessorFeature::AffineBodyStateAccessorFeature(S<AffineBodyStateAccessorFeatureOverrider> overrider)
    : m_impl(std::move(overrider))
{
    UIPC_ASSERT(m_impl, "AffineBodyStateAccessorFeatureOverrider must not be null.");
}

SizeT AffineBodyStateAccessorFeature::body_count() const
{
    return m_impl->get_body_count();
}

geometry::SimplicialComplex AffineBodyStateAccessorFeature::create_geometry(IndexT body_offset,
                                                                             SizeT body_count) {
    auto total_body_num = this->body_count();
    UIPC_ASSERT(body_offset <= total_body_num,
                "body_offset ({}) must not be larger than total body number ({})",
                body_offset,
                total_body_num);

    if(body_count == ~0ull)
        body_count = total_body_num - body_offset;

    return m_impl->do_create_geometry(body_offset, body_count);
}

void AffineBodyStateAccessorFeature::copy_from(const geometry::SimplicialComplex& state_geo) const {
    m_impl->do_copy_from(state_geo);
}

void AffineBodyStateAccessorFeature::copy_to(geometry::SimplicialComplex& state_geo) const {
    m_impl->do_copy_to(state_geo);
}

std::string_view AffineBodyStateAccessorFeature::get_name() const {
    return FeatureName;
}
}  // namespace uipc::core