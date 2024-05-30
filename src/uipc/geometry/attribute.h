#pragma once
#include <string_view>
#include <uipc/common/vector.h>
#include <uipc/common/span.h>
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/backend/buffer_view.h>

namespace uipc::geometry
{
/**
 * @brief An abstract class to represent a geometries attribute.
 */
class IAttribute
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
    void          clear();

    friend backend::BufferView backend_view(const IAttribute& a) noexcept;

  protected:
    backend::BufferView         backend_view() const noexcept;
    virtual SizeT               get_size() const                  = 0;
    virtual backend::BufferView get_backend_view() const noexcept = 0;

    virtual void          do_resize(SizeT N)  = 0;
    virtual void          do_clear()          = 0;
    virtual void          do_reserve(SizeT N) = 0;
    virtual S<IAttribute> do_clone() const    = 0;
};

/**
 * @brief Template class to represent a geometries attribute of type T.
 * 
 * @tparam T The type of the attribute values.
 */
template <typename T>
class Attribute : public IAttribute
{
  public:
    using value_type = T;

    Attribute(const T& default_value = {}) noexcept;

    Attribute(const Attribute<T>&)               = default;
    Attribute(Attribute<T>&&)                    = default;
    Attribute<T>& operator=(const Attribute<T>&) = default;
    Attribute<T>& operator=(Attribute<T>&&)      = default;

    /**
     * @brief Get a non-const view of the attribute values. This method may potentially clone the attribute data.
     * 
     * @note
     *    Always consider using the const member method if the attribute data is not going to be modified.
     * @return `span<T>` 
     */
    friend [[nodiscard]] span<T> view(Attribute<T>& a) noexcept
    {
        return a.m_values;
    }

    /**
     * @brief Get a const view of the attribute values. This method gerantees no data cloning.
     * 
     * @return `span<const T>` 
     */
    [[nodiscard]] span<const T> view() const noexcept;

  protected:
    SizeT               get_size() const override;
    backend::BufferView get_backend_view() const noexcept override;

    void                do_resize(SizeT N) override;
    void                do_clear() override;
    void                do_reserve(SizeT N) override;
    S<IAttribute>       do_clone() const override;

  private:
    backend::BufferView m_backend_view;
    vector<T>           m_values;
    T                   m_default_value;
};
}  // namespace uipc::geometries

#include "details/attribute.inl"