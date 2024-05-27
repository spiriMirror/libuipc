#pragma once
#include <uipc/geometry/simplex_slot.h>
#include <uipc/geometry/attribute_collection.h>

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
}// namespace uipc::geometry

#include "details/simplicial_complex_attributes.inl"
