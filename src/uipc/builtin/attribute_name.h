#pragma once
#include <string_view>

#define UIPC_BUILTIN_ATTRIBUTE(name) constexpr std::string_view name = #name

namespace uipc::builtin
{
/**
 * @brief `position` <Vector3> attribute on **vertices**
 */
UIPC_BUILTIN_ATTRIBUTE(position);

/**
 * @brief `transform` <Matrix4x4> attribute on **instances**
 */
UIPC_BUILTIN_ATTRIBUTE(transform);

/**
 * @brief `contact_element_id` <IndexT> attribute on **meta**
 */

UIPC_BUILTIN_ATTRIBUTE(contact_element_id);

/**
 * @brief `constitution_uid` <U64> attribute on **meta**, uid is a unique identifier for a constitution
 * which is defined in the libuipc specification.
 */
UIPC_BUILTIN_ATTRIBUTE(constitution_uid);

/**
 * @brief `extra_constitution_uids` <VectorXu64> attribute on **meta**, extra constitutions that are applied to the geometry.
 */
UIPC_BUILTIN_ATTRIBUTE(extra_constitution_uids);

/**
 * @brief `implicit_geometry_uid` <U64> attribute on **meta**, uid is a unique identifier for an implicit geometry
 * which is defined in the libuipc specification.
 */
UIPC_BUILTIN_ATTRIBUTE(implicit_geometry_uid);

/**
 * @brief `is_surf` <IndexT> attribute on **vertices** / **edges** / **triangles** / **tetrahedra**... 
 * to indicate if the element is a surface element.
 */
UIPC_BUILTIN_ATTRIBUTE(is_surf);

/**
 * @brief `is_facet` <IndexT> attribute on **vertices** / **edges** / **triangles** / **tetrahedra**
 * to indicate if the element is a facet element.
 */
UIPC_BUILTIN_ATTRIBUTE(is_facet);

/**
 * @brief `orient` <IndexT>[-1,0,1] attribute on **triangles** to indicate the orientation of the triangle.
 * 
 * 1) 0 is the default value, which means the orientation is not determined, or the triangle is not a surface triangle.
 * 2) 1 means outward the tetrahedron
 * 3) -1 means inward the tetrahedron.
 */
UIPC_BUILTIN_ATTRIBUTE(orient);

/**
 * @brief `parent_id` <IndexT> attribute on **edges** / **triangles**, indicates the parent simplex id 
 */
UIPC_BUILTIN_ATTRIBUTE(parent_id);

/**
 * @brief `is_fixed` <IndexT>[0,1] attribute, indicates if the **instance** or **vertex** is fixed.
 * 
 * 1) 0 means the instance or vertex is not fixed.
 * 2) 1 means the instance or vertex is fixed.
 */
UIPC_BUILTIN_ATTRIBUTE(is_fixed);

/**
 * @brief `mass` <Float> attribute on **vertices**.
 */
UIPC_BUILTIN_ATTRIBUTE(mass);

/**
 * @brief `thickness` <Float> attribute on **vertices** to indicate the shell thickness (radius) of the vertices
 * which is valid when dealing with codimensional geometries).
 */
UIPC_BUILTIN_ATTRIBUTE(thickness);


/**
 * @brief `fem_vertex_offset` <IndexT> attribute on **meta** to indicate the offset of the vertex in the FEM system.
 */
UIPC_BUILTIN_ATTRIBUTE(backend_fem_vertex_offset);
}  // namespace uipc::builtin

#undef UIPC_BUILTIN_ATTRIBUTE
