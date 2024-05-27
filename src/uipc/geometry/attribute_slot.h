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
    IAttributeSlot()          = default;
    virtual ~IAttributeSlot() = default;
    // delete copy
    IAttributeSlot(const IAttributeSlot&)            = delete;
    IAttributeSlot& operator=(const IAttributeSlot&) = delete;
    // enable move
    IAttributeSlot(IAttributeSlot&&) noexcept            = default;
    IAttributeSlot& operator=(IAttributeSlot&&) noexcept = default;

    /**
     * @brief Get the name of the attribute slot.
     */
    [[nodiscard]] std::string_view name() const;
    /**
     * @brief Check if the underlying attribute is allowed to be destroyed.
     */
    [[nodiscard]] bool allow_destroy() const;
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
    [[nodiscard]] virtual std::string_view get_name() const          = 0;
    [[nodiscard]] virtual bool             get_allow_destroy() const = 0;

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

    [[nodiscard]] virtual std::string_view get_name() const override;
    [[nodiscard]] virtual bool             get_allow_destroy() const override;

    void                                    do_make_owned() override;
    [[nodiscard]] virtual S<IAttributeSlot> do_clone() const override;

    [[nodiscard]] virtual IAttribute&       get_attribute() override;
    [[nodiscard]] virtual const IAttribute& get_attribute() const override;

    [[nodiscard]] virtual SizeT get_use_count() const override;

  private:
    std::string     m_name;
    S<Attribute<T>> m_attribute;
    bool            m_allow_destroy;
};
}  // namespace uipc::geometry

#include "details/attribute_slot.inl"