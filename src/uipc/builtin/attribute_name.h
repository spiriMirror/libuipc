#pragma once
#include <string_view>
namespace uipc::builtin
{
/**
 * @brief `position` attribute on geometry.vertices();
 */
constexpr std::string_view position = "position";
/**
 * @brief `transform` attribute on geometry.instances()
 */
constexpr std::string_view transform = "transform";
/**
 * @brief `contact_element_id` attribute on geometry.meta()
 */
constexpr std::string_view contact_element_id = "contact_element_id";
/**
 * @brief `constitution` attribute on geometry.meta()
 */
constexpr std::string_view constitution = "constitution";

/**
 * @brief `constitution_uid` attribute on geometry.meta(), uid is a unique identifier for a constitution
 * which is defined in the libuipc specification.
 */
constexpr std::string_view constitution_uid = "constitution_uid";
}  // namespace uipc::builtin
