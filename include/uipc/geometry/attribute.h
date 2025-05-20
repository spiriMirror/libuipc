#pragma once
#include <string_view>
#include <uipc/backend/buffer_view.h>
#include <uipc/common/json.h>
#include <uipc/common/dllexport.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/span.h>
#include <uipc/common/type_define.h>
#include <uipc/common/vector.h>
#include <uipc/geometry/attribute_copy.h>
#include <uipc/common/buffer_info.h>

namespace uipc::geometry
{
/**
 * @brief An abstract class to represent a geometries attribute.
 */
class UIPC_CORE_API IAttribute
{
    template <typename T>
    friend class AttributeFriend;

  public:
    IAttribute()          = default;
    virtual ~IAttribute() = default;

    /**
     * @brief Get the size of the attribute.
     */
    [[nodiscard]] SizeT size() const noexcept;

    [[nodiscard]] Json to_json(SizeT i) const noexcept;

    [[nodiscard]] Json to_json() const noexcept;

    void from_json(const Json& j) noexcept;
    /**
     * @brief Get the type name of data stored in the attribute slot.
     */
    [[nodiscard]] std::string_view type_name() const noexcept;

  private:
    friend class AttributeCollection;
    friend class IAttributeSlot;

    void          resize(SizeT N);
    void          reserve(SizeT N);
    S<IAttribute> clone() const;
    S<IAttribute> clone_empty() const;
    void          clear();
    void          reorder(span<const SizeT> O) noexcept;
    void copy_from(const IAttribute& other, const AttributeCopy& copy) noexcept;

  protected:
    virtual SizeT            get_size() const               = 0;
    virtual std::string_view get_type_name() const noexcept = 0;

    virtual void          do_resize(SizeT N)                       = 0;
    virtual void          do_clear()                               = 0;
    virtual void          do_reserve(SizeT N)                      = 0;
    virtual S<IAttribute> do_clone() const                         = 0;
    virtual S<IAttribute> do_clone_empty() const                   = 0;
    virtual void          do_reorder(span<const SizeT> O) noexcept = 0;
    virtual void do_copy_from(const IAttribute& other, const AttributeCopy& copy) noexcept = 0;

    virtual void do_from_json(const Json& j) noexcept = 0;
    virtual Json do_to_json(SizeT i) const noexcept   = 0;
    virtual Json do_to_json() const noexcept          = 0;
};

template <typename T>
class AttributeSlot;

/** 
 * @brief Template class to represent a geometries attribute of type T.
 * 
 * @tparam T The type of the attribute values.
 */
template <typename T>
class Attribute : public IAttribute
{
    friend class AttributeSlot<T>;

  public:
    using value_type = T;

    Attribute(const T& default_value = {}) noexcept;

    Attribute(const Attribute<T>&)               = default;
    Attribute(Attribute<T>&&)                    = default;
    Attribute<T>& operator=(const Attribute<T>&) = default;
    Attribute<T>& operator=(Attribute<T>&&)      = default;

    friend span<T> view(Attribute<T>& a) noexcept { return a.m_values; }

    [[nodiscard]] span<const T> view() const noexcept;

    [[nodiscard]] static std::string type() noexcept;

  protected:
    virtual SizeT            get_size() const override;
    virtual std::string_view get_type_name() const noexcept override;

    virtual void          do_resize(SizeT N) override;
    virtual void          do_clear() override;
    virtual void          do_reserve(SizeT N) override;
    virtual S<IAttribute> do_clone() const override;
    virtual S<IAttribute> do_clone_empty() const override;
    virtual void          do_reorder(span<const SizeT> O) noexcept override;
    virtual void do_copy_from(const IAttribute& other, const AttributeCopy& copy) noexcept override;

    virtual Json do_to_json(SizeT i) const noexcept override;
    virtual Json do_to_json() const noexcept override;

    virtual void do_from_json(const Json& j) noexcept override;

  private:
    vector<T> m_values;
    T         m_default_value;
};
}  // namespace uipc::geometry

#include "details/attribute.inl"