#pragma once
#include <uipc/backend/visitors/scene_visitor.h>

namespace uipc::backend::cuda
{
class SimEngine;
bool has_affine_body_constitution(SimEngine& engine) noexcept;
}  // namespace uipc::backend::cuda
