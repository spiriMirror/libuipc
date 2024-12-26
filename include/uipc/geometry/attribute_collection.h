#pragma once
#include <uipc/common/string.h>
#include <uipc/common/unordered_map.h>
#include <uipc/common/exception.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/attribute.h>
#include <uipc/geometry/attribute_slot.h>
#include <uipc/geometry/attribute_copy.h>

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
    template <typename T>
    S<AttributeSlot<T>> create(std::string_view name,
                               const T&         default_value = {},
                               bool             allow_destroy = true);

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
    S<IAttributeSlot> share(std::string_view      name,
                            const IAttributeSlot& slot,
                            bool                  allow_destroy = true);

    /**
     * @brief Template version of share.
     */
    template <typename T>
    S<AttributeSlot<T>> share(std::string_view        name,
                              const AttributeSlot<T>& slot,
                              bool                    allow_destroy = true);

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
    [[nodiscard]] S<IAttributeSlot> find(std::string_view name);
    /**
     * @brief const version of find.
     */
    [[nodiscard]] S<const IAttributeSlot> find(std::string_view name) const;

    /**
     * @brief Template version of find.
     */
    template <typename T>
    [[nodiscard]] S<AttributeSlot<T>> find(std::string_view name);

    /**
     * @brief  Template const version of find.
     */
    template <typename T>
    [[nodiscard]] S<const AttributeSlot<T>> find(std::string_view name) const;

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
     * @brief copy_from the underlying attribute values of all attribute slots.
     * 
     * @param copy The copy strategy.
     * @param include_names The names of the attribute slots to be copied. If it is empty, all attribute slots will be copied.
     * @param exclude_names The names of the attribute slots not to be copied, the exclude_names has higher priority than include_names.
     */
    void copy_from(const AttributeCollection& other,
                   const AttributeCopy&       copy,
                   span<const string>         include_names = {},
                   span<const string>         exclude_names = {});

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

    /**
     * @brief Get the names of all attribute slots.
     */
    vector<string> names() const;

    /**
     * @brief Get the number of attribute slots.
    */
    SizeT attribute_count() const;

    /**
    * @brief Get the json representation of the attribute collection.
    */
    Json to_json() const;

  private:
    SizeT                                    m_size = 0;
    unordered_map<string, S<IAttributeSlot>> m_attributes;
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
struct UIPC_CORE_API formatter<uipc::geometry::AttributeCollection>
    : formatter<std::string_view>
{
    appender format(const uipc::geometry::AttributeCollection& collection,
                    format_context&                            ctx);
};
}  // namespace fmt


#include "details/attribute_collection.inl"