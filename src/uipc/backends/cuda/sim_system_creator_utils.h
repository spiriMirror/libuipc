#pragma once
#include <uipc/backend/visitors/scene_visitor.h>

namespace uipc::backend::cuda
{
class SimEngine;
class CreatorQuery
{
  public:
    static bool has_affine_body_constitution(SimEngine& engine) noexcept;
    static bool is_contact_enabled(SimEngine& engine) noexcept;
};
}  // namespace uipc::backend::cuda
