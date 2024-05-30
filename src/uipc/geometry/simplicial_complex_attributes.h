#pragma once
#include <uipc/geometry/simplex_slot.h>
#include <uipc/geometry/attribute_collection.h>

namespace uipc::geometry
{
template <std::derived_from<ISimplexSlot> SimplexSlotT>
class SimplicialComplexAttributes;

template <std::derived_from<ISimplexSlot> SimplexSlotT>
class SimplicialComplexTopo;

/**
* @brief A wrapper of the topology of the simplicial complex.
*/
template <std::derived_from<ISimplexSlot> SimplexSlotT>
class SimplicialComplexTopo
{
    friend class SimplicialComplexAttributes<SimplexSlotT>;

    // Note: SimplexSlotT can be const or non-const
  public:
    /**
     * @brief Get a non-const view of the topology, this function may clone the data.
     */
    template <IndexT N>
    friend span<typename SimplexSlot<N>::ValueT> view(SimplicialComplexTopo<SimplexSlot<N>>&& v);

    /**
     * @brief Get the backend view of the topology, this function guarantees no data clone.
     */
    template <IndexT N>
    friend backend::BufferView backend_view(SimplicialComplexTopo<SimplexSlot<N>>&& v) noexcept;

    template <IndexT N>
    friend backend::BufferView backend_view(SimplicialComplexTopo<const SimplexSlot<N>>&& v) noexcept;

    /**
     * @brief Get a const view of the topology, this function guarantees no data clone.
     */
    [[nodiscard]] auto view() && noexcept { return m_topology.view(); }
    /**
     * @brief Query if the topology is owned by current simplicial complex.
     */
    [[nodiscard]] bool is_shared() && noexcept;

  private:
    SimplicialComplexTopo(SimplexSlotT& topo);
    SimplexSlotT& m_topology;
};

template <>
class SimplicialComplexTopo<VertexSlot>
{
    friend class SimplicialComplexAttributes<VertexSlot>;

  public:
    /**
     * @brief Get a non-const view of the topology, this function may clone the data.
     */
    friend span<typename VertexSlot::ValueT> view(SimplicialComplexTopo<VertexSlot>&& v);

    /**
     * @brief Get the backend view of the topology, this function guarantees no data clone.
     */
    friend backend::BufferView backend_view(SimplicialComplexTopo<VertexSlot>&& v) noexcept;

    /**
     * @brief Get a const view of the topology, this function guarantees no data clone.
     */
    [[nodiscard]] auto view() && noexcept { return m_topology.view(); }
    /**
     * @brief Query if the topology is owned by current simplicial complex.
     */
    [[nodiscard]] bool is_shared() && noexcept;

  private:
    SimplicialComplexTopo(VertexSlot& topo);
    VertexSlot& m_topology;
};

template <>
class SimplicialComplexTopo<const VertexSlot>
{
    friend class SimplicialComplexAttributes<const VertexSlot>;

  public:
    /**
     * @brief Get the backend view of the topology, this function guarantees no data clone.
     */
    friend backend::BufferView backend_view(SimplicialComplexTopo<const VertexSlot>&& v) noexcept;

    /**
     * @brief Get a const view of the topology, this function guarantees no data clone.
     */
    [[nodiscard]] auto view() && noexcept { return m_topology.view(); }

    /**
     * @brief Query if the topology is owned by current simplicial complex.
     */
    [[nodiscard]] bool is_shared() && noexcept;

  private:
    SimplicialComplexTopo(const VertexSlot& topo);
    const VertexSlot& m_topology;
};

/**
 * @brief A collection of attributes for a specific type of simplices. The main API for accessing the attributes of a simplicial complex.
 */
template <std::derived_from<ISimplexSlot> SimplexSlotT>
class SimplicialComplexAttributes
{
    using Topo = SimplicialComplexTopo<SimplexSlotT>;
    using AutoAttributeCollection =
        std::conditional_t<std::is_const_v<SimplexSlotT>, const AttributeCollection, AttributeCollection>;

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
    [[nodiscard]] Topo topo() noexcept;

    [[nodiscard]] Topo topo() const noexcept;

    /**
     * @sa [AttributeCollection::resize()](../AttributeCollection/#resize)
     */
    void resize(SizeT size)
        requires(!std::is_const_v<SimplexSlotT>);
    /**
     * @sa [AttributeCollection::reserve()](../AttributeCollection/#reserve)
     */
    void reserve(SizeT size)
        requires(!std::is_const_v<SimplexSlotT>);
    /**
     * @sa [AttributeCollection::clear()](../AttributeCollection/#clear)
     */
    void clear()
        requires(!std::is_const_v<SimplexSlotT>);
    /**
     * @sa [AttributeCollection::size()](../AttributeCollection/#size)
     */
    [[nodiscard]] SizeT size() const noexcept;
    /**
     * @sa [AttributeCollection::destroy()](../AttributeCollection/#destroy) 
     */
    void destroy(std::string_view name)
        requires(!std::is_const_v<SimplexSlotT>);

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
    SimplexSlotT&            m_topology;
    AutoAttributeCollection& m_attributes;

    SimplicialComplexAttributes(SimplexSlotT&            topology,
                                AutoAttributeCollection& attributes) noexcept;
};
}  // namespace uipc::geometry

#include "details/simplicial_complex_attributes.inl"
