#include <affine_body/affine_body_dynamics.h>


namespace uipc::backend::cuda
{
AffineBodyDynamics::FilteredInfo::FilteredInfo(Impl* impl, SizeT constitution_index) noexcept
    : m_impl(impl)
    , m_constitution_index(constitution_index)
{
}

auto AffineBodyDynamics::FilteredInfo::geo_infos() const noexcept -> span<const GeoInfo>
{

    auto& constitution_info = m_impl->constitution_infos[m_constitution_index];
    return span{m_impl->geo_infos}.subspan(constitution_info.geo_offset,
                                           constitution_info.geo_count);
}

auto AffineBodyDynamics::FilteredInfo::constitution_info() const noexcept
    -> const ConstitutionInfo&
{
    return m_impl->constitution_infos[m_constitution_index];
}

SizeT AffineBodyDynamics::FilteredInfo::body_count() const noexcept
{
    return constitution_info().body_count;
}

SizeT AffineBodyDynamics::FilteredInfo::vertex_count() const noexcept
{
    return constitution_info().vertex_count;
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
}  // namespace uipc::backend::cuda