#include <affine_body/affine_body_constitution.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
U64 AffineBodyConstitution::uid() const
{
    return get_uid();
}

void AffineBodyConstitution::do_build()
{
    m_impl.affine_body_dynamics = require<AffineBodyDynamics>();

    auto scene = world().scene();
    // Check if we have the Affine Body Constitution
    auto uids = scene.constitution_tabular().uids();
    if(!std::binary_search(uids.begin(), uids.end(), uid()))
    {
        throw SimSystemException(fmt::format("Requires Constitution UID={}", uid()));
    }

    BuildInfo info;
    do_build(info);

    m_impl.affine_body_dynamics->add_constitution(this);
}

void AffineBodyConstitution::init(AffineBodyDynamics::FilteredInfo& info)
{
    return do_init(info);
}

void AffineBodyConstitution::compute_energy(ABDLineSearchReporter::ComputeEnergyInfo& info)
{
    ComputeEnergyInfo this_info{&m_impl, m_index, &info};
    return do_compute_energy(this_info);
}

void AffineBodyConstitution::compute_gradient_hessian(ABDLinearSubsystem::ComputeGradientHessianInfo& info)
{
    ComputeGradientHessianInfo this_info{&m_impl, m_index, &info};
    return do_compute_gradient_hessian(this_info);
}

muda::CBufferView<Vector12> AffineBodyConstitution::BaseInfo::qs() const noexcept
{
    auto& abd = m_impl->abd();
    return abd.subview(abd.body_id_to_q, m_index);
}

muda::CBufferView<Vector12> AffineBodyConstitution::BaseInfo::q_prevs() const noexcept
{
    auto& abd = m_impl->abd();
    return abd.subview(abd.body_id_to_q_prev, m_index);
}

muda::CBufferView<Float> AffineBodyConstitution::BaseInfo::volumes() const noexcept
{
    auto& abd = m_impl->abd();
    return abd.subview(abd.body_id_to_volume, m_index);
}


}  // namespace uipc::backend::cuda
