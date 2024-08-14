#include <affine_body/affine_body_dynamics.h>


namespace uipc::backend::cuda
{
AffineBodyDynamics::FilteredInfo::FilteredInfo(Impl* impl) noexcept
    : m_impl(impl)
{
}

AffineBodyDynamics::ComputeEnergyInfo::ComputeEnergyInfo(Impl* impl,
                                                         SizeT constitution_index,
                                                         Float dt) noexcept
    : m_impl(impl)
    , m_constitution_index(constitution_index)
    , m_dt(dt)
{
}

AffineBodyDynamics::ComputeGradientHessianInfo::ComputeGradientHessianInfo(
    Impl*                         impl,
    SizeT                         constitution_index,
    muda::BufferView<Vector12>    shape_gradient,
    muda::BufferView<Matrix12x12> shape_hessian,
    Float                         dt) noexcept
    : m_impl(impl)
    , m_constitution_index(constitution_index)
    , m_shape_gradient(shape_gradient)
    , m_shape_hessian(shape_hessian)
    , m_dt(dt)
{
}

U64 AffineBodyDynamics::BodyInfo::constitution_uid() const noexcept
{
    return m_constitution_uid;
}
IndexT AffineBodyDynamics::BodyInfo::geometry_slot_index() const noexcept
{
    return m_geometry_slot_index;
}
IndexT AffineBodyDynamics::BodyInfo::geometry_instance_index() const noexcept
{
    return m_geometry_instance_index;
}
IndexT AffineBodyDynamics::BodyInfo::abd_geometry_index() const noexcept
{
    return m_abd_geometry_index;
}
IndexT AffineBodyDynamics::BodyInfo::affine_body_id() const noexcept
{
    return m_affine_body_id;
}
IndexT AffineBodyDynamics::BodyInfo::vertex_offset() const noexcept
{
    return m_vertex_offset;
}
IndexT AffineBodyDynamics::BodyInfo::vertex_count() const noexcept
{
    return m_vertex_count;
}

muda::CBufferView<Vector12> AffineBodyDynamics::ComputeEnergyInfo::qs() const noexcept
{
    return m_impl->subview(m_impl->body_id_to_q, m_constitution_index);
}
muda::CBufferView<Float> AffineBodyDynamics::ComputeEnergyInfo::volumes() const noexcept
{
    return m_impl->subview(m_impl->body_id_to_volume, m_constitution_index);
}
muda::BufferView<Float> AffineBodyDynamics::ComputeEnergyInfo::body_shape_energies() const noexcept
{
    return m_impl->subview(m_impl->body_id_to_shape_energy, m_constitution_index);
}
muda::CBufferView<Vector12> AffineBodyDynamics::ComputeGradientHessianInfo::qs() const noexcept
{
    return m_impl->subview(m_impl->body_id_to_q, m_constitution_index);
}
muda::CBufferView<Float> AffineBodyDynamics::ComputeGradientHessianInfo::volumes() const noexcept
{
    return m_impl->subview(m_impl->body_id_to_volume, m_constitution_index);
}

auto AffineBodyDynamics::FilteredInfo::body_infos() const noexcept -> span<const BodyInfo>
{
    return m_impl->subview(m_impl->h_body_infos, m_constitution_index);
}

geometry::SimplicialComplex& AffineBodyDynamics::FilteredInfo::geometry(
    span<S<geometry::GeometrySlot>> geo_slots, const BodyInfo& body_info)
{
    return Impl::geometry(geo_slots, body_info);
}
}  // namespace uipc::backend::cuda