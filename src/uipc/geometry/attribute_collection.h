#pragma once
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/attribute.h>
#include <map>

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

    std::string_view name() const;
    bool             is_shared() const;
    SizeT            size() const;

  protected:
    friend class AttributeCollection;

    void         make_owned();
    virtual void do_make_owned() = 0;

    SizeT         use_count() const;
    virtual SizeT get_use_count() const = 0;

    virtual U<IAttributeSlot> clone() const;
    virtual U<IAttributeSlot> do_clone() const = 0;

    virtual IAttribute&       attribute();
    virtual IAttribute&       get_attribute() = 0;
    virtual const IAttribute& attribute() const;
    virtual const IAttribute& get_attribute() const = 0;

  private:
    std::string m_name;
};

template <typename T>
class AttributeSlot final : public IAttributeSlot
{
  public:
    using value_type = T;

    AttributeSlot(std::string_view m_name, S<Attribute<T>> attribute);

    std::span<T>       view();
    std::span<const T> view() const;

  protected:
    friend class AttributeCollection;

    void                      do_make_owned() override;
    virtual U<IAttributeSlot> do_clone() const override;

    virtual IAttribute&       get_attribute() override;
    virtual const IAttribute& get_attribute() const override;

    virtual SizeT get_use_count() const override;

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

    IAttributeSlot*       find(std::string_view name);
    const IAttributeSlot* find(std::string_view name) const;

    template <typename T>
    AttributeSlot<T>* find(std::string_view name);

    template <typename T>
    const AttributeSlot<T>* find(std::string_view name) const;

    void   resize(size_t N);
    size_t size() const;
    void   clear();
    void   reserve(size_t N);

  private:
    size_t                                   m_size = 0;
    std::map<std::string, U<IAttributeSlot>> m_attributes;
};

class AttributeAlreadyExist : public std::exception
{
  public:
    AttributeAlreadyExist(std::string_view msg);

    const char* what() const noexcept override;

  private:
    std::string m_msg;
};
}  // namespace uipc::geometry

#include "details/attribute_collection.inl"