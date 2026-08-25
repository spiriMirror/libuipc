#pragma once
#include <uipc/common/type_define.h>
#include <uipc/geometry/attribute_collection.h>

namespace uipc::core
{
// Scene Default Config
geometry::AttributeCollection default_scene_config() noexcept;
Json                          scene_config_schema();
void validate_scene_config(const geometry::AttributeCollection& config);

// Util Functions:

Json to_config_json(const geometry::AttributeCollection& config);
void from_config_json(geometry::AttributeCollection& config, const Json& j);
}  // namespace uipc::core
