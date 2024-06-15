#pragma once
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/attribute.h>
#include <map>
#include <uipc/common/exception.h>
#include <uipc/backend/buffer_view.h>

namespace uipc::geometry
{
class AttributeCollection;
/**
 * @brief An abstract class to represent a geometries attribute slot in a geometries attribute collection.
 * 
 */
class UIPC_CORE_API IAttributeSlot
{
  public:
    IAttributeSlot()          = default;
    virtual ~IAttributeSlot() = default;
    // delete copy_from
    IAttributeSlot(const IAttributeSlot&)            = delete;
    IAttributeSlot& operator=(const IAttributeSlot&) = delete;
    // enable move
    IAttributeSlot(IAttributeSlot&&) noexcept            = default;
    IAttributeSlot& operator=(IAttributeSlot&&) noexcept = default;

    /**
     * @brief Get the name of the attribute slot.
     */
    [[nodiscard]] std::string_view name() const noexcept;
    /**
     * @brief Get the type name of data stored in the attribute slot.
     */
    [[nodiscard]] std::string_view type_name() const noexcept;
    /**
     * @brief Check if the underlying attribute is allowed to be destroyed.
     */
    [[nodiscard]] bool allow_destroy() const noexcept;
    /**
     * @brief Check if the underlying attribute is shared.
     * 
     * @return true, if the underlying attribute is shared, more than one geometries reference to the underlying attribute.
     * @return false, if the underlying attribute is owned, only this geometries reference to the underlying attribute. 
     */
    [[nodiscard]] bool  is_shared() const noexcept;
    [[nodiscard]] SizeT size() const noexcept;

    friend backend::BufferView backend_view(const IAttributeSlot&) noexcept;

  protected:
    friend class AttributeCollection;
    [[nodiscard]] virtual std::string_view get_name() const noexcept = 0;
    [[nodiscard]] virtual bool get_allow_destroy() const noexcept    = 0;

    void         make_owned();
    virtual void do_make_owned() = 0;

    [[nodiscard]] SizeT         use_count() const;
    [[nodiscard]] virtual SizeT get_use_count() const = 0;

    [[nodiscard]] virtual std::string_view  get_type_name() const noexcept = 0;
    [[nodiscard]] virtual S<IAttributeSlot> clone() const;
    [[nodiscard]] virtual S<IAttributeSlot> do_clone() const = 0;
    [[nodiscard]] virtual S<IAttributeSlot> clone_empty() const;
    [[nodiscard]] virtual S<IAttributeSlot> do_clone_empty() const = 0;

    [[nodiscard]] virtual IAttribute&       attribute() noexcept;
    [[nodiscard]] virtual IAttribute&       get_attribute() noexcept = 0;
    [[nodiscard]] virtual const IAttribute& attribute() const noexcept;
    [[nodiscard]] virtual const IAttribute& get_attribute() const noexcept = 0;
};

/**
 * @brief Template class to represent a geometries attribute slot of type T in a geometries attribute collection.
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
     */
    template <typename U>
    friend span<U> view(AttributeSlot<U>& slot);
    /**
     * @brief Get the const attribute values.
     * 
     * @return `span<const T>`
     */
    [[nodiscard]] span<const T> view() const noexcept;

  protected:
    friend class AttributeCollection;

    [[nodiscard]] virtual std::string_view get_type_name() const noexcept override;
    [[nodiscard]] virtual std::string_view get_name() const noexcept override;
    [[nodiscard]] virtual bool get_allow_destroy() const noexcept override;

    [[nodiscard]] virtual IAttribute& get_attribute() noexcept override;
    [[nodiscard]] virtual const IAttribute& get_attribute() const noexcept override;
    [[nodiscard]] virtual SizeT get_use_count() const noexcept override;


    void                                    do_make_owned() override;
    [[nodiscard]] virtual S<IAttributeSlot> do_clone() const override;
    [[nodiscard]] virtual S<IAttributeSlot> do_clone_empty() const override;

  private:
    std::string     m_name;
    S<Attribute<T>> m_attribute;
    bool            m_allow_destroy;
};
}  // namespace uipc::geometry

#include "details/attribute_slot.inl"