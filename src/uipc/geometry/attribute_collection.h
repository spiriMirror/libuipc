#pragma once
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/attribute.h>
#include <map>
#include <uipc/common/exception.h>

namespace uipc::geometry
{
class AttributeCollection;
/**
 * @brief An abstract class to represent a geometry attribute slot in a geometry attribute collection.
 * 
 */
class IAttributeSlot
{
  public:
    virtual ~IAttributeSlot() = default;
    IAttributeSlot(std::string_view m_name, bool allow_destroy = true);
    // delete copy
    IAttributeSlot(const IAttributeSlot&)            = delete;
    IAttributeSlot& operator=(const IAttributeSlot&) = delete;
    // enable move
    IAttributeSlot(IAttributeSlot&&) noexcept            = default;
    IAttributeSlot& operator=(IAttributeSlot&&) noexcept = default;

    [[nodiscard]] std::string_view name() const;
    /**
     * @brief Check if the underlying attribute is shared.
     * 
     * @return true, if the underlying attribute is shared, more than one geometry reference to the underlying attribute.
     * @return false, if the underlying attribute is owned, only this geometry reference to the underlying attribute. 
     */
    [[nodiscard]] bool  is_shared() const;
    [[nodiscard]] SizeT size() const;
  protected:
    friend class AttributeCollection;

    void         make_owned();
    virtual void do_make_owned() = 0;

    [[nodiscard]] SizeT         use_count() const;
    [[nodiscard]] virtual SizeT get_use_count() const = 0;

    [[nodiscard]] virtual S<IAttributeSlot> clone() const;
    [[nodiscard]] virtual S<IAttributeSlot> do_clone() const = 0;

    [[nodiscard]] virtual IAttribute&       attribute();
    [[nodiscard]] virtual IAttribute&       get_attribute() = 0;
    [[nodiscard]] virtual const IAttribute& attribute() const;
    [[nodiscard]] virtual const IAttribute& get_attribute() const = 0;

  private:
    std::string m_name;

  protected:
    bool        m_allow_destroy;
};

/**
 * @brief Template class to represent a geometry attribute slot of type T in a geometry attribute collection.
 * 
 * @tparam T The type of the attribute values.
 */
template <typename T>
class AttributeSlot final : public IAttributeSlot
{
  public:
    using value_type = T;

    AttributeSlot(std::string_view m_name, S<Attribute<T>> attribute, bool allow_destroy);

    /**
     * @brief Get the non-const attribute values.
     * 
     * @return `span<T>`
     * @sa [Attribute](../Attribute/index.md#Attribute)
     */
    template <typename U>
    friend span<U> view(AttributeSlot<U>& slot);
    /**
     * @brief Get the const attribute values.
     * 
     * @return `span<const T>`
     */
    [[nodiscard]] span<const T> view() const;

  protected:
    friend class AttributeCollection;

    void                                    do_make_owned() override;
    [[nodiscard]] virtual S<IAttributeSlot> do_clone() const override;

    [[nodiscard]] virtual IAttribute&       get_attribute() override;
    [[nodiscard]] virtual const IAttribute& get_attribute() const override;

    [[nodiscard]] virtual SizeT get_use_count() const override;

  private:
    S<Attribute<T>> m_attribute;
};

/**
 * @brief A collection of geometry attributes.
 *
 * All geometry attributes in the collection always have the same size.
 */
class AttributeCollection
{
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
     * The slot may be from another geometry attribute collection or just current geometry attribute collection.
     * @param name The name of the attribute slot.
     * @param slot The slot brings the underlying attribute.
     * @return The new created attribute slot.
     *
     * @throw AttributeAlreadyExist if the attribute with the given name already exists.
     */
    IAttributeSlot& share(std::string_view name, const IAttributeSlot& slot);

    /**
     * @brief Template version of share.
     */
    template <typename T>
    AttributeSlot<T>& share(std::string_view name, const AttributeSlot<T>& slot);

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
    void resize(size_t N);
    /**
     * @brief Get the size of the attribute slots.
     */
    [[nodiscard]] size_t size() const;
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
    void reserve(size_t N);

  private:
    size_t                                   m_size = 0;
    std::map<std::string, S<IAttributeSlot>> m_attributes;
};

class AttributeAlreadyExist : public Exception
{
  public:
    using Exception::Exception;
};

class AttributeDontAllowDestroy : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::geometry

#include "details/attribute_collection.inl"