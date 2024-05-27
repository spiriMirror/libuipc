#pragma once
#include <uipc/geometry/simplex_slot.h>

namespace uipc::geometry
{
/**
 * @brief Represents an abstract simplicial complex, containing vertices, edges, triangles, and tetrahedra.
 * 
 * @note Abstract simplicial complex does not contain any geometric information, such as coordinates of vertices.
 */
class AbstractSimplicialComplex
{
    friend class SimplicialComplex;

  public:
    AbstractSimplicialComplex();

    AbstractSimplicialComplex(const AbstractSimplicialComplex&);
    AbstractSimplicialComplex& operator=(const AbstractSimplicialComplex&);
    AbstractSimplicialComplex(AbstractSimplicialComplex&&) noexcept;
    AbstractSimplicialComplex& operator=(AbstractSimplicialComplex&&) noexcept;

    /**
     * @brief Get the non-const slot for vertices.
     * 
     * @return a non-const slot for vertices
     */
    VertexSlot& vertices();
    /**
     * @brief Get the const slot for vertices.
     * 
     * @return a const slot for vertices
     */
    const VertexSlot& vertices() const;
    /**
     * @brief Get the non-const slot for edges.
     * 
     * @return a non-const slot for edges
     */
    EdgeSlot& edges();
    /**
     * @brief Get the const slot for edges.
     * 
     * @return a const slot for edges
     */
    const EdgeSlot& edges() const;
    /**
     * @brief Get the non-const slot for triangles.
     * 
     * @return a non-const slot for triangles
     */
    TriangleSlot& triangles();
    /**
     * @brief Get the const slot for triangles.
     * 
     * @return a const slot for triangles
     */
    const TriangleSlot& triangles() const;
    /**
     * @brief Get the non-const slot for tetrahedra.
     * 
     * @return a non-const slot for tetrahedra
     */
    TetrahedronSlot& tetrahedra();
    /**
     * @brief Get the const slot for tetrahedra.
     * 
     * @return a const slot for tetrahedra
     */
    const TetrahedronSlot& tetrahedra() const;

  private:
    VertexSlot      m_vertices;
    EdgeSlot        m_edges;
    TriangleSlot    m_triangles;
    TetrahedronSlot m_tetrahedra;
};
}  // namespace uipc::geometry