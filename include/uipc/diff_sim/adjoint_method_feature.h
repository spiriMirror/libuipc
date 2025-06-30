#pragma once
#include <uipc/core/feature.h>
#include <uipc/backend/buffer_view.h>

namespace uipc::diff_sim
{
class UIPC_CORE_API AdjointMethodFeatureOverrider
{
  public:
    virtual void do_select_dofs(SizeT frame, backend::BufferView in_SDI) = 0;
    virtual void do_receive_dofs(backend::BufferView out_dofs)           = 0;
    virtual void do_compute_dLdP(backend::BufferView out_dLdP,
                                 backend::BufferView in_dLdx)            = 0;
};

class UIPC_CORE_API AdjointMethodFeature final : public core::Feature
{
    class Impl;

  public:
    constexpr static std::string_view FeatureName = "diff_sim/adjoint_method";

    AdjointMethodFeature(S<AdjointMethodFeatureOverrider> overrider);

    /**
     * @brief Selects the degrees of freedom from the given indices for the given frame.
     * 
     * `in_SDI` is frame-local, this function should be called for each frame to select the degrees of freedom.
     * 
     * @param[in] in_SDI:BufferView<IndexT> The indices of the degrees of freedom to select.
     */
    void select_dofs(SizeT frame, backend::BufferView in_SDI);

    /**
     * @brief Receives the selected degrees of freedom for all frames.
     * 
     * This function should be called after all frames have been processed to receive the selected degrees of freedom.
     * 
     * @param[out] out_dofs:BufferView<Float> The selected degrees of freedom.
     */
    void receive_dofs(backend::BufferView out_Dofs);

    /**
     * @brief Computes the gradient of the loss with respect to the parameters.
     * 
     * @param[out] out_dLdP:BufferView<Float> The gradient of the loss with respect to the parameters.
     * @param[in] in_dLdX:BufferView<Float> The gradient of the loss with respect to the degrees of freedom.
     */
    void compute_dLdP(backend::BufferView out_dLdP, backend::BufferView in_dLdX);

  private:
    virtual std::string_view         get_name() const final override;
    S<AdjointMethodFeatureOverrider> m_impl;
    SizeT                            last_calling_frame = 0;
};
}  // namespace uipc::diff_sim
