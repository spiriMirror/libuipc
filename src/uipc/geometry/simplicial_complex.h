#pragma once
#include <uipc/common/span.h>
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/simplices.h>
#include <uipc/geometry/attribute_collection.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/abstract_simplicial_complex.h>
#include <uipc/common/log.h>

namespace uipc::geometry
{
/**
 * @brief A collection of attributes for a specific type of simplices. The main API for accessing the attributes of a simplicial complex.
 */
template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
class SimplicialComplexAttributes
{
  public:
    SimplicialComplexAttributes(const SimplicialComplexAttributes& o) = default;
    SimplicialComplexAttributes(SimplicialComplexAttributes&& o)      = default;
    SimplicialComplexAttributes& operator=(const SimplicialComplexAttributes& o) = default;
    SimplicialComplexAttributes& operator=(SimplicialComplexAttributes&& o) = default;

    void                resize(size_t size);
    void                reserve(size_t size);
    void                clear();
    [[nodiscard]] SizeT size() const;

    /**
     * @brief Get a non-const view of the topology.
     * 
     * @warning This function may cause a data clone if the topology is shared.
     */
    [[nodiscard]] auto topo_view() { return m_topology->view(); }
    /**
     * @brief Get a const view of the topology, this function guarantees no data clone.
     */
    [[nodiscard]] auto topo_view() const
    {
        return std::as_const(m_topology)->view();
    }
    /**
     * @brief Query if the topology is owned by current simplicial complex.
     */
    [[nodiscard]] bool topo_is_shared() const;

    /**
     * @brief Find an attribute by type and name, if the attribute does not exist, return nullptr.
     */
    template <typename T>
    [[nodiscard]] auto find(std::string_view name)
    {
        return m_attributes.template find<T>(name);
    }

    template <typename T>
    decltype(auto) create(std::string_view name)
    {
        return m_attributes.template create<T>(name);
    }

  private:
    friend class SimplicialComplex;
    SimplexSlot&         m_topology;
    AttributeCollection& m_attributes;

    SimplicialComplexAttributes(SimplexSlot& topology, AttributeCollection& attributes);
};

/**
 * @brief A simplicial complex is a collection of simplices.
 * 
 * In $\mathbb{R}^3$, a simplicial complex is a collection of vertices, edges, triangles, and tetrahedra, such that
 * $$
 * K = (V, E, F, T),
 * $$
 * where $V$ is the set of vertices, $E$ is the set of edges,
 * $F$ is the set of triangles, and $T$ is the set of tetrahedra.
 * 
 * In addition to the topology, a simplicial complex have at least one attribute called "position", 
 * representing the positions of the vertices. 
 * 
 * Because "position" is so common, we provide a short cut to get the positions of the vertices, as shown below:
 *
 * ```cpp
 * auto mesh = geometry::tetmesh(Vs, Ts);
 * auto& attr_pos = mesh.positions();
 * ```
 * @note To use `geometry::tetmesh`, you need to `#include <uipc/geometry/factory.h>`.
 * @sa [geometry::tetmesh()](../index.md#tetmesh)
 * 
 *
 * We can also use the generic API to get the positions(or any other attributes) of the vertices, as shown below:
 * ```cpp
 * auto VA = mesh.vertices();
 * auto pos = VA.find<Vector3>("position");
 * ```
 *
 * To the underlying attributes of the simplicial complex, we need to create a view of the attributes, as shown below:
 * ```cpp
 * // const view
 * std::as_const(pos)->view();
 * // non-const view
 * pos->view();
 * ```
 * @tip A non-const view of the attribute may cause data clone if the attribute is shared.
 * If you don't tend to modify the attribute, always use the const version of the view. 
 * @danger Never store a view of any attribute, because the view may become invalid after the attribute is modified. 
 * Always create a new view when you need it. Don't mind, the view is lightweight. :white_check_mark:
 *
 * To get the tetrahedra of the simplicial complex, we can use the following code:
 * ```cpp
 *  auto TA  = mesh.tetrahedra();
 * ```
 * You may want to add new attributes to the tetrahedra, for example, 
 * in [Continuum Mechanics](https://en.wikipedia.org/wiki/Continuum_mechanics), 
 * the deformation gradient $\mathbf{F} \in \mathbb{R}^{3\times 3}$ is an important attribute of the tetrahedra.
 * We can add the attribute to the tetrahedra as shown below:
 * ```cpp
 * auto& F = TA.create<Matrix3x3>("F");
 * auto F_view = F.view();
 * std::fill(F_view.begin(), F_view.end(), Matrix3x3::Identity());
 * ```
 * @sa [Tutorial/Geometry](../../../../tutorial/geometry.md)
 */
class SimplicialComplex : public IGeometry
{
  public:
    using VertexAttributes      = SimplicialComplexAttributes<VertexSlot>;
    using EdgeAttributes        = SimplicialComplexAttributes<EdgeSlot>;
    using TriangleAttributes    = SimplicialComplexAttributes<TriangleSlot>;
    using TetrahedronAttributes = SimplicialComplexAttributes<TetrahedronSlot>;

    SimplicialComplex(const AbstractSimplicialComplex& asc, span<const Vector3> positions);

    SimplicialComplex(const SimplicialComplex& o)            = default;
    SimplicialComplex(SimplicialComplex&& o)                 = default;
    SimplicialComplex& operator=(const SimplicialComplex& o) = default;
    SimplicialComplex& operator=(SimplicialComplex&& o)      = default;

    /**
     * @brief Get the positions of the vertices
     * 
     * @return AttributeSlot<Vector3>& 
     */
    [[nodiscard]] AttributeSlot<Vector3>& positions();

    /**
     * @brief A short cut to get the positions of the vertices
     * 
     * @return const AttributeSlot<Vector3>& 
     */
    [[nodiscard]] const AttributeSlot<Vector3>& positions() const;

    /**
     * @brief A wrapper of the vertices and its attributes of the simplicial complex.
     * 
     * @return VertexAttributes 
     */
    [[nodiscard]] VertexAttributes vertices();

    /**
     * @brief A wrapper of the edges and its attributes of the simplicial complex.
     * 
     * @return EdgeAttributes 
     */
    [[nodiscard]] EdgeAttributes edges();
    /**
     * @brief  A wrapper of the triangles and its attributes of the simplicial complex.
     * 
     * @return TriangleAttributes 
     */
    [[nodiscard]] TriangleAttributes triangles();
    /**
     * @brief A wrapper of the tetrahedra and its attributes of the simplicial complex.
     * 
     * @return TetrahedronAttributes 
     */
    [[nodiscard]] TetrahedronAttributes tetrahedra();

    /**
     * @brief Get the dimension of the simplicial complex.
     *
     * Return the maximum dimension of the simplices in the simplicial complex.
     * 
     * @return IndexT 
     */
    [[nodiscard]] IndexT dim() const;

  protected:
    virtual std::string_view get_type() const override;

  private:
    AbstractSimplicialComplex m_asc;
    AttributeCollection       m_vertex_attributes;
    AttributeCollection       m_edge_attributes;
    AttributeCollection       m_triangle_attributes;
    AttributeCollection       m_tetrahedron_attributes;
};
}  // namespace uipc::geometry

#include "details/simplicial_complex.inl"