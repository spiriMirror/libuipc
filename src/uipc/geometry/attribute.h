#pragma once
#include <string_view>
#include <uipc/common/vector.h>
#include <uipc/common/span.h>
#include <uipc/common/type_define.h>
#include <uipc/common/macro.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/backend/buffer_view.h>

namespace uipc::geometry
{
/**
 * @brief An abstract class to represent a geometries attribute.
 */
class UIPC_CORE_API IAttribute
{
  public:
    IAttribute() = default;
    /**
     * @brief Get the size of the attribute.
     */
    [[nodiscard]] SizeT size() const;

  private:
    friend class AttributeCollection;
    friend class IAttributeSlot;
    void          resize(SizeT N);
    void          reserve(SizeT N);
    S<IAttribute> clone() const;
    S<IAttribute> clone_empty() const;
    void          clear();
    void          reorder(span<const SizeT> O) noexcept;
    void copy_from(const IAttribute& other, span<const SizeT> O) noexcept;

    friend backend::BufferView backend_view(const IAttribute& a) noexcept;

  protected:
    backend::BufferView         backend_view() const noexcept;
    virtual SizeT               get_size() const                  = 0;
    virtual backend::BufferView get_backend_view() const noexcept = 0;

    virtual void          do_resize(SizeT N)                       = 0;
    virtual void          do_clear()                               = 0;
    virtual void          do_reserve(SizeT N)                      = 0;
    virtual S<IAttribute> do_clone() const                         = 0;
    virtual S<IAttribute> do_clone_empty() const                   = 0;
    virtual void          do_reorder(span<const SizeT> O) noexcept = 0;
    virtual void do_copy_from(const IAttribute& other, span<const SizeT> O) noexcept = 0;
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

    friend [[nodiscard]] span<T> view(Attribute<T>& a) noexcept
    {
        return a.m_values;
    }

    [[nodiscard]] span<const T> view() const noexcept;

  protected:
    SizeT               get_size() const override;
    backend::BufferView get_backend_view() const noexcept override;

    void          do_resize(SizeT N) override;
    void          do_clear() override;
    void          do_reserve(SizeT N) override;
    S<IAttribute> do_clone() const override;
    S<IAttribute> do_clone_empty() const override;
    void          do_reorder(span<const SizeT> O) noexcept override;
    void do_copy_from(const IAttribute& other, span<const SizeT> O) noexcept override;

  private:
    backend::BufferView m_backend_view;
    vector<T>           m_values;
    T                   m_default_value;
};
}  // namespace uipc::geometry

#include "details/attribute.inl"