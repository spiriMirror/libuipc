#pragma once
#include <uipc/core/feature.h>
#include <uipc/common/smart_pointer.h>

namespace uipc::diff_sim
{
class UIPC_CORE_API EnableGradFeatureOverrider
{
  public:
    virtual void do_no_grad()                = 0;
    virtual void do_with_grad()              = 0;
    virtual bool get_is_grad_enabled() const = 0;
};

class UIPC_CORE_API EnableGradFeature final : public core::Feature
{
  public:
    static constexpr std::string_view FeatureName = "diff_sim/enable_grad";

    EnableGradFeature(S<EnableGradFeatureOverrider> overrider);

    /**
     * @brief With DiffSim on, turn off parameter gradient computation.
     */
    void no_grad();

    /**
     * @brief With DiffSim on, turn on parameter gradient computation.
     */
    void with_grad();

    /**
    * @brief Check if parameter gradient computation is enabled.
    * @return true if parameter gradient computation is enabled.
    */
    bool is_grad_enabled() const;

  private:
    virtual std::string_view      get_name() const override final;
    S<EnableGradFeatureOverrider> m_impl;
};
}  // namespace uipc::diff_sim
