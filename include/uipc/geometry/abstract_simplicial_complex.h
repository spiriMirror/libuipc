#pragma once
#include <uipc/geometry/simplicial_complex_attributes.h>
#include <uipc/geometry/geometry.h>

namespace uipc::geometry
{
/**
 * @brief Represents an abstract simplicial complex, containing vertices, edges, triangles, and tetrahedra.
 * 
 * @note Abstract simplicial complex does not contain any geometric information, such as coordinates of vertices.
 */
class UIPC_CORE_API AbstractSimplicialComplex : public Geometry
{
    template <typename T>
    friend class AttributeFriend;

  public:
    /**
     * @brief Alias for the vertex attributes
     * 
     * @sa SimplicialComplexAttributes
     */
    using VertexAttributes  = SimplicialComplexAttributes<false, 0>;
    using CVertexAttributes = SimplicialComplexAttributes<true, 0>;
    /**
     * @brief Alias for the edge attributes
     * 
     * @sa SimplicialComplexAttributes
     */
    using EdgeAttributes  = SimplicialComplexAttributes<false, 1>;
    using CEdgeAttributes = SimplicialComplexAttributes<true, 1>;
    /**
     * @brief Alias for the triangle attributes
     * 
     * @sa SimplicialComplexAttributes
     */
    using TriangleAttributes  = SimplicialComplexAttributes<false, 2>;
    using CTriangleAttributes = SimplicialComplexAttributes<true, 2>;
    /**
     * @brief Alias for the tetrahedron attributes
     *
     * @sa SimplicialComplexAttributes
     */
    using TetrahedronAttributes  = SimplicialComplexAttributes<false, 3>;
    using CTetrahedronAttributes = SimplicialComplexAttributes<true, 3>;

    AbstractSimplicialComplex()                                   = default;
    AbstractSimplicialComplex(const AbstractSimplicialComplex& o) = default;
    AbstractSimplicialComplex(AbstractSimplicialComplex&& o)      = default;

    AbstractSimplicialComplex& operator=(const AbstractSimplicialComplex& o) = delete;
    AbstractSimplicialComplex& operator=(AbstractSimplicialComplex&& o) = delete;

    /**
     * @brief A wrapper of the vertices and its attributes of the simplicial complex.
     * 
     * @return VertexAttributeInfo 
     */
    [[nodiscard]] VertexAttributes  vertices() noexcept;
    [[nodiscard]] CVertexAttributes vertices() const noexcept;

    /**
     * @brief A wrapper of the edges and its attributes of the simplicial complex.
     * 
     * @return EdgeAttributes 
     */
    [[nodiscard]] EdgeAttributes  edges() noexcept;
    [[nodiscard]] CEdgeAttributes edges() const noexcept;


    /**
     * @brief  A wrapper of the triangles and its attributes of the simplicial complex.
     * 
     * @return TriangleAttributes 
     */
    [[nodiscard]] TriangleAttributes  triangles() noexcept;
    [[nodiscard]] CTriangleAttributes triangles() const noexcept;
    /**
     * @brief A wrapper of the tetrahedra and its attributes of the simplicial complex.
     * 
     * @return TetrahedronAttributes 
     */
    [[nodiscard]] TetrahedronAttributes  tetrahedra() noexcept;
    [[nodiscard]] CTetrahedronAttributes tetrahedra() const noexcept;

    /**
     * @brief Get the dimension of the simplicial complex.
     *
     * Return the maximum dimension of the simplices in the simplicial complex.
     * 
     * @return IndexT 
     */
    [[nodiscard]] IndexT dim() const noexcept;


  protected:
    virtual std::string_view   get_type() const noexcept override;
    [[nodiscard]] virtual Json do_to_json() const override;

  private:
    friend struct fmt::formatter<AbstractSimplicialComplex>;
    AttributeCollection m_vertex_attributes;
    AttributeCollection m_edge_attributes;
    AttributeCollection m_triangle_attributes;
    AttributeCollection m_tetrahedron_attributes;
};
}  // namespace uipc::geometry

namespace fmt
{
template <>
struct UIPC_CORE_API formatter<uipc::geometry::AbstractSimplicialComplex>
    : formatter<string_view>
{
    appender format(const uipc::geometry::AbstractSimplicialComplex& c,
                    format_context&                                  ctx) const;
};
}  // namespace fmt
