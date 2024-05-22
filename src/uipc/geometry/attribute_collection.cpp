#include <uipc/geometry/attribute_collection.h>
#include <uipc/common/log.h>
namespace uipc::geometry
{
std::string_view IAttributeSlot::name() const
{
    return m_name;
}

bool IAttributeSlot::is_owned() const
{
    // if I find that no other object is using this attribute, then I can own it
    return use_count() == 1;
}

IAttributeSlot::IAttributeSlot(std::string_view m_name)
    : m_name(m_name)
{
}

SizeT IAttributeSlot::size() const
{
    return attribute().size();
}

void IAttributeSlot::make_owned()
{
    if(is_owned())
        return;
    do_make_owned();
}

SizeT IAttributeSlot::use_count() const
{
    return get_use_count();
}

U<IAttributeSlot> IAttributeSlot::clone() const
{
    return do_clone();
}

IAttribute& IAttributeSlot::attribute()
{
    return get_attribute();
}

const IAttribute& IAttributeSlot::attribute() const
{
    return get_attribute();
}

const char* AttributeAlreadyExist::what() const noexcept
{
    return m_msg.c_str();
}

AttributeAlreadyExist::AttributeAlreadyExist(std::string_view name)
{
    m_msg = "Attribute with name " + std::string(name) + " already exist";
}

IAttributeSlot& AttributeCollection::share(std::string_view name, const IAttributeSlot& slot)
{
    auto n  = std::string{name};
    auto it = m_attributes.find(n);
    if(it != m_attributes.end())
        throw AttributeAlreadyExist{name};
    return *(m_attributes[n] = slot.clone());
}

void AttributeCollection::destroy(std::string_view name)
{
    auto it = m_attributes.find(std::string{name});
    if(it == m_attributes.end())
        return;
    m_attributes.erase(it);
}

IAttributeSlot* AttributeCollection::find(std::string_view name)
{
    auto it = m_attributes.find(std::string{name});
    if(it == m_attributes.end())
        return nullptr;
    return it->second.get();
}


const IAttributeSlot* AttributeCollection::find(std::string_view name) const
{
    auto it = m_attributes.find(std::string{name});
    if(it == m_attributes.end())
        return nullptr;
    return it->second.get();
}

void AttributeCollection::resize(size_t N)
{
    for(auto& [name, slot] : m_attributes)
    {
        slot->make_owned();
        slot->attribute().resize(N);
    }
    m_size = N;
}

size_t AttributeCollection::size() const
{
    return m_size;
}

void AttributeCollection::clear()
{
    for(auto& [name, slot] : m_attributes)
    {
        slot->make_owned();
        slot->attribute().clear();
    }
}
void AttributeCollection::reserve(size_t N)
{
    for(auto& [name, slot] : m_attributes)
    {
        slot->attribute().reserve(N);
    }
}
AttributeCollection::AttributeCollection(const AttributeCollection& o)
{
    for(auto& [name, attr] : o.m_attributes)
    {
        m_attributes[name] = attr->clone();
    }
    m_size = o.m_size;
}
AttributeCollection& AttributeCollection::operator=(const AttributeCollection& o)
{
    if(std::addressof(o) == this)
        return *this;
    for(auto& [name, attr] : o.m_attributes)
    {
        m_attributes[name] = attr->clone();
    }
    m_size = o.m_size;
}
AttributeCollection::AttributeCollection(AttributeCollection&& o) noexcept
    : m_attributes(std::move(o.m_attributes))
    , m_size(o.m_size)
{
    o.m_size = 0;
}
AttributeCollection& AttributeCollection::operator=(AttributeCollection&& o) noexcept
{
    if(std::addressof(o) == this)
        return *this;
    m_attributes = std::move(o.m_attributes);
    m_size       = o.m_size;
    o.m_size     = 0;
    return *this;
}

}  // namespace uipc::geometry
