#pragma once
#include <uipc/common/unordered_map.h>
#include <uipc/common/exception.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/attribute.h>
#include <uipc/geometry/attribute_slot.h>

namespace uipc::geometry
{
/**
 * @brief A collection of geometries attributes.
 *
 * All geometries attributes in the collection always have the same size.
 */
class UIPC_CORE_API AttributeCollection
{
    friend struct fmt::formatter<AttributeCollection>;

  public:
    AttributeCollection() = default;

    AttributeCollection(const AttributeCollection&);
    AttributeCollection& operator=(const AttributeCollection&);

    AttributeCollection(AttributeCollection&&) noexcept;
    AttributeCollection& operator=(AttributeCollection&&) noexcept;

    /**
     * @brief Create a new attribute slot of type T with a given name.
     * 
     * @tparam T The type of the attribute values.
     * @param name The name of the attribute slot.
     * @return The created attribute slot.
     */
    template <typename T, bool AllowDestroy = true>
    P<AttributeSlot<T>> create(std::string_view name, const T& default_value = {});

    /**
     * @brief Share the underlying attribute of the given slot with a new name.
     * 
     * The slot may be from another geometries attribute collection or just current geometries attribute collection.
     * @param name The name of the attribute slot.
     * @param slot The slot brings the underlying attribute.
     * @return The new created attribute slot.
     *
     * @throw AttributeAlreadyExist if the attribute with the given name already exists.
     */
    P<IAttributeSlot> share(std::string_view name, const IAttributeSlot& slot);

    /**
     * @brief Template version of share.
     */
    template <typename T>
    P<AttributeSlot<T>> share(std::string_view name, const AttributeSlot<T>& slot);

    /**
     * @brief Remove the attribute slot with the given name.
     *
     * The underlying attribute will not be destroyed if it is shared by other attribute slots.
     * 
     * @danger Accessing the removed attribute slot will cause undefined behavior. 
     * It's user's responsibility to ensure that the removed attribute slot is not accessed.
     * @param name 
     */
    void destroy(std::string_view name);

    /**
     * @brief Find the attribute slot with the given name.
     * 
     * @param name The name of the attribute slot.
     * @return The attribute slot with the given name.
     * @return nullptr if the attribute slot with the given name does not exist.
     */
    [[nodiscard]] P<IAttributeSlot> find(std::string_view name);
    /**
     * @brief const version of find.
     */
    [[nodiscard]] P<const IAttributeSlot> find(std::string_view name) const;

    /**
     * @brief Template version of find.
     */
    template <typename T>
    [[nodiscard]] P<AttributeSlot<T>> find(std::string_view name);

    /**
     * @brief  Template const version of find.
     */
    template <typename T>
    [[nodiscard]] P<const AttributeSlot<T>> find(std::string_view name) const;

    /**
     * @brief Resize all attribute slots to the given size.
     * 
     * @note This method may generate data clones.
     */
    void resize(SizeT N);

    /**
	 * @brief Reorder the underlying attribute values of all attribute slots.
	 * 
	 * @param O A New2Old mapping. O[i] = j means the i-th element in the new order has the value of the j-th element in the old order.
	 * 
	 * @note This method may generate data clones.
	 */
    void reorder(span<const SizeT> O);

    /**
     * @brief Get the size of the attribute slots.
     */
    [[nodiscard]] SizeT size() const;
    /**
     * @brief clear the underlying attribute values of all attribute slots, the attribute slots will not be destroyed.
     * 
     * @note This method may generate data clones.
     */
    void clear();
    /**
     * @brief Reserve memory for all attribute slots.
     *
     * @note This method generates no data clone. But the memory of the underlying attribute values may be reallocated.
     */
    void reserve(SizeT N);

  private:
    SizeT                                         m_size = 0;
    unordered_map<std::string, S<IAttributeSlot>> m_attributes;
};

class UIPC_CORE_API GeometryAttributeError : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::geometry


namespace fmt
{
template <>
struct formatter<uipc::geometry::AttributeCollection> : formatter<std::string_view>
{
    appender format(const uipc::geometry::AttributeCollection& collection,
                    format_context&                            ctx);
};

extern template struct UIPC_CORE_API formatter<uipc::geometry::AttributeCollection>;
}  // namespace fmt


#include "details/attribute_collection.inl"