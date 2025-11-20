#pragma once
#include <uipc/common/type_define.h>
#include <uipc/geometry/attribute_collection.h>

namespace uipc::core
{
// Scene Default Config
geometry::AttributeCollection default_scene_config() noexcept;

// Util Functions:

Json to_config_json(const geometry::AttributeCollection& config);
void from_config_json(geometry::AttributeCollection& config, const Json& j);
}  // namespace uipc::core