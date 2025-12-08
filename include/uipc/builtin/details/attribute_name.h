// Don't put #pragma once here, this file may be included multiple times.

/*****************************************************************/ /**
 * @file   attribute_name.h
 * @brief  This file records all the built-in attribute names in the libuipc specification.
 * 
 * Programmers can define their own UIPC_BUILTIN_ATTRIBUTE macro outside this file, and include this file to get the built-in attribute names.
 * 
 * @code{.cpp}
 * #define UIPC_BUILTIN_ATTRIBUTE(name) constexpr std::string_view name = #name
 * #include <uipc/builtin/attribute_name.h>
 * #undef UIPC_BUILTIN_ATTRIBUTE
 * @endcode
 * 
 * @author MuGdxy
 * @date   September 2024
 *********************************************************************/

#ifdef UIPC_BUILTIN_ATTRIBUTE
/**
 * @brief `topo` attribute on **vertices** / **edges** / **triangles** / **tetrahedra**... 
 * to indicate the topological type of the element.
 */
UIPC_BUILTIN_ATTRIBUTE(topo);

/**
 * @brief `position` <Vector3> attribute on **vertices**
 */
UIPC_BUILTIN_ATTRIBUTE(position);

/**
 * @brief `velocity` <Vector3> attribute on **vertices** or `velocity` <Matrix4x4> attribute on **instances** (the derivative of the transform matrix).
 */
UIPC_BUILTIN_ATTRIBUTE(velocity);

/**
 * @brief `aim_position` <Vector3> attribute on **vertices**, indicates the aim position of the vertices if the vertices are animated.
 */
UIPC_BUILTIN_ATTRIBUTE(aim_position);

/**
 * @brief `transform` <Matrix4x4> attribute on **instances**
 */
UIPC_BUILTIN_ATTRIBUTE(transform);

/**
 * @brief `aim_transform` <Matrix4x4> attribute on **instances**, indicates the aim transform of the instances if the instances are animated.
 */
UIPC_BUILTIN_ATTRIBUTE(aim_transform);

/**
 * @brief `contact_element_id` <IndexT> attribute on **meta**
 */

UIPC_BUILTIN_ATTRIBUTE(contact_element_id);

/**
 * @brief `subscene_element_id` <IndexT> attribute on **meta**
 */

UIPC_BUILTIN_ATTRIBUTE(subscene_element_id);

/**
 * @brief `d_hat` <Float> attribute on **meta**, indicates mesh-wise d_hat
 */

UIPC_BUILTIN_ATTRIBUTE(d_hat);

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
 * @brief `constraint_uids` <VectorXu64> attribute on **meta**,  constraint uids are unique identifiers for constraints
 * which is defined in the libuipc specification.
 */
UIPC_BUILTIN_ATTRIBUTE(constraint_uids);

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
 * 'Fixed' means the vertices are not influenced by its constitution and kinetic, keeping the position unchanged.
 * They may be fixed with a certain position, or they are animated with a certain aim position.
 * 
 * 1) 0 means the instance or vertex is not fixed.
 * 2) 1 means the instance or vertex is fixed.
 */
UIPC_BUILTIN_ATTRIBUTE(is_fixed);

/**
 * @brief `is_constrained` <IndexT>[0,1] attribute, indicates if the **instance** or **vertex** is constrained.
 * 
 * 'Constrained' means the instances or vertices are trying to obey the constraints.
 * 
 * 1) 0 means the instance or vertex is not constrained.
 * 2) 1 means the instance or vertex is constrained.
 */
UIPC_BUILTIN_ATTRIBUTE(is_constrained);

/**
 * @brief `is_dynamic` <IndexT>[0,1] attribute, indicates if the **instance** or **vertex** is dynamic.
 * 
 * 'Dynamic' means the kinetic of instances or vertices is considered.
 * 
 * 1) 0 means the the kinetic of the instance or vertex is not considered.
 * 2) 1 means the the kinetic of the instance or vertex is considered.
 */
UIPC_BUILTIN_ATTRIBUTE(is_dynamic);

/**
 * @brief `external_kinetic` <IndexT>[0,1] attribute，indicates if the **instance** or **vertex** is fully controlled by external kinetic.
 * 
 * If an instance or vertex is marked with `external_kinetic` attribute, default kinetic computation will be skipped for it.
 * It's always used together with Constraints to fully control the kinetic of the instance or vertex.
 * 
 * 1) 0 means the instance or vertex will go through default kinetic computation.
 * 2) 1 means the instance or vertex will skip default kinetic computation.
 */
UIPC_BUILTIN_ATTRIBUTE(external_kinetic);

/**
 * @brief `volume` <Float> attribute on **vertices**.
 */
UIPC_BUILTIN_ATTRIBUTE(volume);

/**
 * @brief `mass_density` <Float> attribute on **vertices** or **meta**.
 */
UIPC_BUILTIN_ATTRIBUTE(mass_density);

/**
 * @brief `gravity` <Vector3> attribute on **instance** or **vertices**.
 * 
 * If attribute exists, backend will take this value as the gravity of the instance or vertices,
 * instead of the global gravity set in the scene configuration.
 */
UIPC_BUILTIN_ATTRIBUTE(gravity);

/**
 * @brief `thickness` <Float> attribute on **vertices** to indicate the shell thickness (radius) of the vertices
 * which is valid when dealing with codimensional geometries.
 */
UIPC_BUILTIN_ATTRIBUTE(thickness);

/**
 * @brief `backend_fem_vertex_offset` <IndexT> attribute on **meta** to indicate the offset of the vertex in the FEM system.
 */
UIPC_BUILTIN_ATTRIBUTE(backend_fem_vertex_offset);

/**
 * @brief `backend_abd_body_offset` <IndexT> attribute on **meta** to indicate the offset of the body(instance) in the ABD system.
 */
UIPC_BUILTIN_ATTRIBUTE(backend_abd_body_offset);

/**
 * @brief `dof_offset` <IndexT> attribute on **meta** to indicate the degree of freedom offset of this geometry in the whole system.
 */
UIPC_BUILTIN_ATTRIBUTE(dof_offset);

/**
 * @brief `dof_count` <IndexT> attribute on **meta** to indicate the degree of freedom count of this geometry in the whole system.
 */
UIPC_BUILTIN_ATTRIBUTE(dof_count);

/**
 * @breif `global_vertex_offset` <IndexT> attribute on **meta** to indicate the global vertex offset of this geometry in the whole system.
 */
UIPC_BUILTIN_ATTRIBUTE(global_vertex_offset);

/**
 * @brief `self_collision` <IndexT>[0,1] attribute on **meta** to indicate if the geometry needs self-collision detection.
 */
UIPC_BUILTIN_ATTRIBUTE(self_collision);
#endif
