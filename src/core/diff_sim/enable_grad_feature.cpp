#include <uipc/diff_sim/enable_grad_feature.h>

namespace uipc::diff_sim
{
EnableGradFeature::EnableGradFeature(S<EnableGradFeatureOverrider> overrider)
    : m_impl(std::move(overrider))
{
    UIPC_ASSERT(m_impl, "NoGradFeatureOverrider must not be null.");
}

void EnableGradFeature::no_grad()
{
    m_impl->do_no_grad();
}

void EnableGradFeature::with_grad()
{
    m_impl->do_with_grad();
}

bool EnableGradFeature::is_grad_enabled() const
{
    return m_impl->get_is_grad_enabled();
}
std::string_view uipc::diff_sim::EnableGradFeature::get_name() const
{
    return FeatureName;
}
}  // namespace uipc::diff_sim
