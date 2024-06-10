#pragma once
#include <string_view>
namespace uipc::builtin
{
/**
 * @brief `position` attribute on geometries.vertices();
 */
constexpr std::string_view position = "position";
/**
 * @brief `transform` attribute on geometries.instances()
 */
constexpr std::string_view transform = "transform";
/**
 * @brief `contact_element_id` attribute on geometries.meta()
 */
constexpr std::string_view contact_element_id = "contact_element_id";
/**
 * @brief `constitution` attribute on geometries.meta()
 */
constexpr std::string_view constitution = "constitution";

/**
 * @brief `constitution_uid` attribute on geometries.meta(), uid is a unique identifier for a constitution
 * which is defined in the libuipc specification.
 */
constexpr std::string_view constitution_uid = "constitution_uid";

/**
 * @brief `implicit_geometry_uid` attribute on geometries.meta(), uid is a unique identifier for an implicit geometry
 * which is defined in the libuipc specification.
 */
constexpr std::string_view implicit_geometry_uid = "implicit_geometry_uid";


/**
 * @brief `is_surf` attribute on vertices/edges/triangles/tetrahedra... to indicate if the element is a surface element.
 */
constexpr std::string_view is_surf = "is_surf";

/**
 * @brief `orient` (value=-1,0,1) attribute on triangles to indicate the orientation of the triangle.
 * 
 * 1) 0 is the default value, which means the orientation is not determined, or the triangle is not a surface triangle.
 * 2) 1 means outward the tetrahedron
 * 3) -1 means inward the tetrahedron.
 */
constexpr std::string_view orient = "orient";

/**
 * .@brief `parent_id` attribute, indicates the parent simplex id 
 */
constexpr std::string_view parent_id = "parent_id";
}  // namespace uipc::builtin
