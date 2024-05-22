#pragma once
#include <string_view>
#include <vector>
#include <span>
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>

namespace uipc::geometry
{
class IAttribute
{
  public:
    IAttribute() = default;

    SizeT size() const;

    template <typename Derived>
        requires std::is_base_of_v<IAttribute, Derived>
    Derived& cast();

    template <typename Derived>
        requires std::is_base_of_v<IAttribute, Derived>
    const Derived& cast() const;

  private:
    friend class AttributeCollection;
    friend class IAttributeSlot;
    void          resize(SizeT N);
    void          reserve(SizeT N);
    S<IAttribute> clone() const;
    void          clear();

  protected:
    virtual SizeT         get_size() const    = 0;
    virtual void          do_resize(SizeT N)  = 0;
    virtual void          do_clear()          = 0;
    virtual void          do_reserve(SizeT N) = 0;
    virtual S<IAttribute> do_clone() const    = 0;
};

template <typename T>
class Attribute : public IAttribute
{
  public:
    using value_type = T;

    Attribute() = default;

    Attribute(const Attribute<T>&)               = default;
    Attribute(Attribute<T>&&)                    = default;
    Attribute<T>& operator=(const Attribute<T>&) = default;
    Attribute<T>& operator=(Attribute<T>&&)      = default;

    std::span<T>       view();
    std::span<const T> view() const;

  protected:
    SizeT         get_size() const override;
    void          do_resize(SizeT N) override;
    void          do_clear() override;
    void          do_reserve(SizeT N) override;
    S<IAttribute> do_clone() const override;

  private:
    std::vector<T> m_values;
};
}  // namespace uipc::geometry

#include "details/attribute.inl"