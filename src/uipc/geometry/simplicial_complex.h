#pragma once
#include <uipc/common/span.h>
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/simplices.h>
#include <uipc/geometry/attribute_collection.h>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/abstract_simplicial_complex.h>
#include <uipc/common/log.h>

namespace uipc::geometry::details
{
}

namespace uipc::geometry
{
template <typename SimplexSlotT>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlotT>
class SimplicialComplexAttributes;

template <typename SimplexSlotT>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlotT>
class SimplicialComplexTopo;

/**
* @brief A wrapper of the topology of the simplicial complex.
*/
template <typename SimplexSlotT>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlotT>
class SimplicialComplexTopo
{
    friend class SimplicialComplexAttributes<SimplexSlotT>;

  public:
    static constexpr IndexT Dimension = SimplexSlotT::Dimension;

    using ValueT = typename SimplexSlotT::ValueT;

    template <IndexT N>
    friend span<typename SimplexSlot<N>::ValueT> view(SimplicialComplexTopo<SimplexSlot<N>>&& v);

    /**
     * @brief Get a const view of the topology, this function guarantees no data clone.
     */
    [[nodiscard]] auto view() && { return m_topology.view(); }
    /**
     * @brief Query if the topology is owned by current simplicial complex.
     */
    [[nodiscard]] bool is_shared() && { return m_topology.is_shared(); }

  private:
    SimplicialComplexTopo(SimplexSlotT& topo);
    SimplexSlotT& m_topology;
};

template <>
class SimplicialComplexTopo<VertexSlot>
{
    friend class SimplicialComplexAttributes<VertexSlot>;
    using ThisT = SimplicialComplexTopo<VertexSlot>;

  public:
    static constexpr IndexT Dimension = VertexSlot::Dimension;

    using ValueT = typename VertexSlot::ValueT;


    friend span<IndexT> view(ThisT&&);

    /**
     * @brief Get a const view of the topology, this function guarantees no data clone.
     */
    [[nodiscard]] auto view() && { return m_topology.view(); }
    /**
     * @brief Query if the topology is owned by current simplicial complex.
     */
    [[nodiscard]] bool is_shared() && { return m_topology.is_shared(); }

  private:
    SimplicialComplexTopo(VertexSlot& topo);
    VertexSlot& m_topology;
};

/**
 * @brief A collection of attributes for a specific type of simplices. The main API for accessing the attributes of a simplicial complex.
 */
template <typename SimplexSlotT>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlotT>
class SimplicialComplexAttributes
{
    using Topo = SimplicialComplexTopo<SimplexSlotT>;

  public:
    SimplicialComplexAttributes(const SimplicialComplexAttributes& o) = default;
    SimplicialComplexAttributes(SimplicialComplexAttributes&& o)      = default;
    SimplicialComplexAttributes& operator=(const SimplicialComplexAttributes& o) = default;
    SimplicialComplexAttributes& operator=(SimplicialComplexAttributes&& o) = default;


    /**
	 * @brief Get the topology of the simplicial complex.
	 * 
	 * @return Topo 
	 */
    [[nodiscard]] Topo topo();
    /**
     * @sa [AttributeCollection::resize()](../AttributeCollection/#resize)
     */
    void resize(SizeT size);
    /**
     * @sa [AttributeCollection::reserve()](../AttributeCollection/#reserve)
     */
    void reserve(SizeT size);
    /**
     * @sa [AttributeCollection::clear()](../AttributeCollection/#clear)
     */
    void clear();
    /**
     * @sa [AttributeCollection::size()](../AttributeCollection/#size)
     */
    [[nodiscard]] SizeT size() const;
    /**
     * @sa [AttributeCollection::destroy()](../AttributeCollection/#destroy) 
     */
    void destroy(std::string_view name);

    /**
     * @brief Find an attribute by type and name, if the attribute does not exist, return nullptr.
     */
    template <typename T>
    [[nodiscard]] decltype(auto) find(std::string_view name)
    {
        return m_attributes.template find<T>(name);
    }

    /**
    * @brief Find an attribute by type and name, if the attribute does not exist, return nullptr.
    */
    template <typename T>
    [[nodiscard]] decltype(auto) find(std::string_view name) const
    {
        return std::as_const(m_attributes).template find<T>(name);
    }

    template <typename T>
    decltype(auto) create(std::string_view name, const T& default_value = {})
    {
        return m_attributes.template create<T>(name, default_value);
    }

    template <typename T>
    decltype(auto) share(std::string_view name, const AttributeSlot<T>& slot)
    {
        return m_attributes.template share<T>(name, slot);
    }

  private:
    friend class SimplicialComplex;
    Topo                 m_topology;
    AttributeCollection& m_attributes;

    SimplicialComplexAttributes(SimplexSlotT& topology, AttributeCollection& attributes);
};

/**
 * @brief A simplicial complex is a collection of simplices.
 * 
 * In $\mathbb{R}^3$, a simplicial complex is defined as follows:
 * $$
 * K = (V, E, F, T),
 * $$
 * where $V$ is the set of vertices, $E$ is the set of edges, $F$ is the set of triangles, and $T$ is the set of tetrahedra.
 * 
 * @sa [Tutorial/Geometry](../../../../tutorial/geometry.md)
 */
class SimplicialComplex : public Geometry
{
  public:
    /**
     * @brief Alias for the vertex attributes
     * 
     * @sa [SimplicialComplexAttributes](../../SimplicialComplexAttributes/)
     */
    using VertexAttributes = SimplicialComplexAttributes<VertexSlot>;
    /**
     * @brief Alias for the edge attributes
     * 
     * @sa [SimplicialComplexAttributes](../../SimplicialComplexAttributes/)
     */
    using EdgeAttributes = SimplicialComplexAttributes<EdgeSlot>;
    /**
     * @brief Alias for the triangle attributes
     * 
     * @sa [SimplicialComplexAttributes](../../SimplicialComplexAttributes/)
     */
    using TriangleAttributes = SimplicialComplexAttributes<TriangleSlot>;
    /**
     * @brief Alias for the tetrahedron attributes
     *
     * @sa [SimplicialComplexAttributes](../../SimplicialComplexAttributes/)
     */
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