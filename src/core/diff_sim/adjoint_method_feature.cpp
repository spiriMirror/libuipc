#include <uipc/diff_sim/adjoint_method_feature.h>
#include <uipc/core/diff_sim.h>
#include <Eigen/Core>
#include <magic_enum.hpp>
#include <uipc/common/timer.h>
#include <uipc/backend/buffer_view.h>
#include <uipc/common/zip.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/core/scene.h>

namespace uipc::diff_sim
{
AdjointMethodFeature::AdjointMethodFeature(S<AdjointMethodFeatureOverrider> overrider)
    : m_impl{overrider}
{
    UIPC_ASSERT(overrider, "Overrider must not be null");
}

std::string_view AdjointMethodFeature::get_name() const
{
    return FeatureName;
}

void AdjointMethodFeature::select_dofs(SizeT frame, backend::BufferView in_SDI)
{
    UIPC_ASSERT(frame >= 1, "Frame must be >= 1, but it is {}", frame);

    if(frame > 1)
    {
        UIPC_ASSERT(last_calling_frame == frame - 1,
                    "`select_dofs` must be called in each frame in order. Last calling frame = {},"
                    " but this calling frame = {}",
                    last_calling_frame,
                    frame);
    }

    last_calling_frame = frame;

    m_impl->do_select_dofs(frame, in_SDI);
}

void AdjointMethodFeature::receive_dofs(backend::BufferView out_Dofs)
{
    m_impl->do_receive_dofs(out_Dofs);
}

void AdjointMethodFeature::compute_dLdP(backend::BufferView out_dLdP, backend::BufferView in_dLdX)
{
    m_impl->do_compute_dLdP(out_dLdP, in_dLdX);
}
}  // namespace uipc::diff_sim
