#pragma once
#include <uipc/geometry/attribute_slot.h>
#include <uipc/geometry/attribute_collection.h>
#include <uipc/common/json.h>
#include <uipc/geometry/attribute_friend.h>
namespace uipc::geometry
{
template <bool IsConst, IndexT N>
class SimplicialComplexAttributes
{
  public:
    friend class fmt::formatter<SimplicialComplexAttributes<IsConst, N>>;

    template <typename T>
    friend class AttributeFriend;

    template <bool OtherIsConst, IndexT OtherN>
    friend class SimplicialComplexAttributes;

    static constexpr bool IsScalar = (N == 0);

    using AutoAttributeCollection =
        std::conditional_t<IsConst, const AttributeCollection, AttributeCollection>;

    using TopoValueT = std::conditional_t<IsScalar, IndexT, Vector<IndexT, N + 1>>;

    SimplicialComplexAttributes(const SimplicialComplexAttributes& o) = default;
    SimplicialComplexAttributes(SimplicialComplexAttributes&& o)      = default;
    SimplicialComplexAttributes& operator=(const SimplicialComplexAttributes& o) = default;
    SimplicialComplexAttributes& operator=(SimplicialComplexAttributes&& o) = default;

    template <bool OtherIsConst>
    SimplicialComplexAttributes(const SimplicialComplexAttributes<OtherIsConst, N>& o) noexcept
        requires(!OtherIsConst)
        : m_attributes(o.m_attributes)
    {
    }

    /**
	 * @brief Get the topology of the simplicial complex.
	 * 
	 * @return Topo 
	 */
    [[nodiscard]] AttributeSlot<TopoValueT>& topo()
        requires(!IsConst && N > 0)
    {
        return *m_attributes.template find<TopoValueT>("topo");
    }

    [[nodiscard]] const AttributeSlot<TopoValueT>& topo() const
        requires(N > 0)
    {
        return *m_attributes.template find<TopoValueT>("topo");
    }

    /**
     * @sa AttributeCollection::resize
     */
    void resize(SizeT size)
        requires(!IsConst)
    {
        m_attributes.resize(size);
    }

    /**
     * @sa AttributeCollection::reserve
     */
    void reserve(SizeT size)
        requires(!IsConst)
    {
        m_attributes.reserve(size);
    }
    /**
     * @sa AttributeCollection::clear
     */
    void clear()
        requires(!IsConst)
    {
        m_attributes.clear();
    }
    /**
     * @sa AttributeCollection::size
     */
    [[nodiscard]] SizeT size() const noexcept { return m_attributes.size(); }
    /**
     * @sa AttributeCollection::destroy
     */
    void destroy(std::string_view name)
        requires(!IsConst)
    {
        m_attributes.destroy(name);
    }

    /**
     * @brief Find an attribute by type and name, if the attribute does not exist, return nullptr.
     */
    template <typename T>
    [[nodiscard]] decltype(auto) find(std::string_view name)
        requires(!IsConst)
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
    decltype(auto) create(std::string_view name, const T& default_value = {}, bool allow_destroy = true)
        requires(!IsConst)
    {
        return m_attributes.template create<T>(name, default_value, allow_destroy);
    }

    template <typename T>
    decltype(auto) share(std::string_view name, const AttributeSlot<T>& slot, bool allow_destroy = true)
        requires(!IsConst)
    {
        return m_attributes.template share<T>(name, slot, allow_destroy);
    }

    void copy_from(SimplicialComplexAttributes<true, N> other,
                   const AttributeCopy&                 copy          = {},
                   span<const string>                   include_names = {},
                   span<const string>                   exclude_names = {})
        requires(!IsConst)
    {
        m_attributes.copy_from(other.m_attributes, copy, include_names, exclude_names);
    }

    Json to_json() const { return m_attributes.to_json(); }

  private:
    friend class SimplicialComplex;
    friend class AbstractSimplicialComplex;
    AutoAttributeCollection& m_attributes;

    SimplicialComplexAttributes(AutoAttributeCollection& attributes) noexcept
        : m_attributes(attributes)
    {
    }
};

extern template class SimplicialComplexAttributes<false, 0>;
extern template class SimplicialComplexAttributes<true, 0>;

extern template class SimplicialComplexAttributes<false, 1>;
extern template class SimplicialComplexAttributes<true, 1>;

extern template class SimplicialComplexAttributes<false, 2>;
extern template class SimplicialComplexAttributes<true, 2>;

extern template class SimplicialComplexAttributes<false, 3>;
extern template class SimplicialComplexAttributes<true, 3>;
}  // namespace uipc::geometry


namespace fmt
{
template <bool IsConst, uipc::IndexT N>
struct formatter<uipc::geometry::SimplicialComplexAttributes<IsConst, N>>
    : formatter<std::string_view>
{
    appender format(const uipc::geometry::SimplicialComplexAttributes<IsConst, N>& attributes,
                    format_context& ctx) const
    {
        return ::fmt::format_to(ctx.out(), "{}", attributes.m_attributes);
    }
};
}  // namespace fmt