#pragma once
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/attribute.h>
#include <map>
#include <uipc/common/exception.h>

namespace uipc::geometry
{
class AttributeCollection;

class IAttributeSlot
{
  public:
    virtual ~IAttributeSlot() = default;
    IAttributeSlot(std::string_view m_name);
    // delete copy
    IAttributeSlot(const IAttributeSlot&)            = delete;
    IAttributeSlot& operator=(const IAttributeSlot&) = delete;
    // enable move
    IAttributeSlot(IAttributeSlot&&) noexcept            = default;
    IAttributeSlot& operator=(IAttributeSlot&&) noexcept = default;

    [[nodiscard]] std::string_view name() const;
    [[nodiscard]] bool             is_shared() const;
    [[nodiscard]] SizeT            size() const;

  protected:
    friend class AttributeCollection;

    void         make_owned();
    virtual void do_make_owned() = 0;

    [[nodiscard]] SizeT         use_count() const;
    [[nodiscard]] virtual SizeT get_use_count() const = 0;

    [[nodiscard]] virtual U<IAttributeSlot> clone() const;
    [[nodiscard]] virtual U<IAttributeSlot> do_clone() const = 0;

    [[nodiscard]] virtual IAttribute&       attribute();
    [[nodiscard]] virtual IAttribute&       get_attribute() = 0;
    [[nodiscard]] virtual const IAttribute& attribute() const;
    [[nodiscard]] virtual const IAttribute& get_attribute() const = 0;

  private:
    std::string m_name;
};

template <typename T>
class AttributeSlot final : public IAttributeSlot
{
  public:
    using value_type = T;

    AttributeSlot(std::string_view m_name, S<Attribute<T>> attribute);

    [[nodiscard]] std::span<T>       view();
    [[nodiscard]] std::span<const T> view() const;

  protected:
    friend class AttributeCollection;

    void                                    do_make_owned() override;
    [[nodiscard]] virtual U<IAttributeSlot> do_clone() const override;

    [[nodiscard]] virtual IAttribute&       get_attribute() override;
    [[nodiscard]] virtual const IAttribute& get_attribute() const override;

    [[nodiscard]] virtual SizeT get_use_count() const override;

  private:
    S<Attribute<T>> m_attribute;
};

class AttributeCollection
{
  public:
    AttributeCollection() = default;

    AttributeCollection(const AttributeCollection&);
    AttributeCollection& operator=(const AttributeCollection&);

    AttributeCollection(AttributeCollection&&) noexcept;
    AttributeCollection& operator=(AttributeCollection&&) noexcept;

    template <typename T>
    AttributeSlot<T>& create(std::string_view name);

    IAttributeSlot& share(std::string_view name, const IAttributeSlot& slot);

    template <typename T>
    AttributeSlot<T>& share(std::string_view name, const AttributeSlot<T>& slot);

    void destroy(std::string_view name);

    [[nodiscard]] IAttributeSlot*       find(std::string_view name);
    [[nodiscard]] const IAttributeSlot* find(std::string_view name) const;

    template <typename T>
    [[nodiscard]] AttributeSlot<T>* find(std::string_view name);

    template <typename T>
    [[nodiscard]] const AttributeSlot<T>* find(std::string_view name) const;

    void                 resize(size_t N);
    [[nodiscard]] size_t size() const;
    void                 clear();
    void                 reserve(size_t N);

  private:
    size_t                                   m_size = 0;
    std::map<std::string, U<IAttributeSlot>> m_attributes;
};

class AttributeAlreadyExist : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::geometry

#include "details/attribute_collection.inl"