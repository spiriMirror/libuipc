#pragma once
#include <string_view>

#define UIPC_BUILTIN_ATTRIBUTE(name) constexpr std::string_view name = #name

namespace uipc::builtin
{
/**
 * @brief `position` attribute on geometries.vertices();
 */
UIPC_BUILTIN_ATTRIBUTE(position);

/**
 * @brief `transform` attribute on geometries.instances()
 */
UIPC_BUILTIN_ATTRIBUTE(transform);

/**
 * @brief `contact_element_id` attribute on geometries.meta()
 */

UIPC_BUILTIN_ATTRIBUTE(contact_element_id);

/**
 * @brief `constitution_uid` attribute on geometries.meta(), uid is a unique identifier for a constitution
 * which is defined in the libuipc specification.
 */
UIPC_BUILTIN_ATTRIBUTE(constitution_uid);

/**
 * @brief `implicit_geometry_uid` attribute on geometries.meta(), uid is a unique identifier for an implicit geometry
 * which is defined in the libuipc specification.
 */
UIPC_BUILTIN_ATTRIBUTE(implicit_geometry_uid);

/**
 * @brief `is_surf` attribute on vertices/edges/triangles/tetrahedra... to indicate if the element is a surface element.
 */
UIPC_BUILTIN_ATTRIBUTE(is_surf);

/**
 * @brief `orient` (value=-1,0,1) attribute on triangles to indicate the orientation of the triangle.
 * 
 * 1) 0 is the default value, which means the orientation is not determined, or the triangle is not a surface triangle.
 * 2) 1 means outward the tetrahedron
 * 3) -1 means inward the tetrahedron.
 */
UIPC_BUILTIN_ATTRIBUTE(orient);

/**
 * @brief `parent_id` attribute, indicates the parent simplex id 
 */
UIPC_BUILTIN_ATTRIBUTE(parent_id);

/**
 * @brief `is_fixed` attribute, indicates if the instance or vertex is fixed.
 */
UIPC_BUILTIN_ATTRIBUTE(is_fixed);

/**
 * @brief `mass_density` attribute
 */
UIPC_BUILTIN_ATTRIBUTE(mass_density);

/**
 * @brief `mass` attribute on vertices/edges/triangles/tetrahedra... to indicate the mass of the element.
 */
UIPC_BUILTIN_ATTRIBUTE(mass);
}  // namespace uipc::builtin

#undef UIPC_BUILTIN_ATTRIBUTE
