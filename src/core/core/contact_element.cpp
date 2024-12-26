#include <uipc/core/contact_element.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::core
{
ContactElement::ContactElement(IndexT id, std::string_view name) noexcept
    : m_id{id}
    , m_name{name}
{
}

IndexT ContactElement::id() const noexcept
{
    return m_id;
}

std::string_view ContactElement::name() const noexcept
{
    return m_name;
}

S<geometry::AttributeSlot<IndexT>> ContactElement::apply_to(geometry::Geometry& geo) const
{
    auto slot = geo.meta().find<IndexT>(builtin::contact_element_id);
    if(!slot)
    {
        slot = geo.meta().create<IndexT>(builtin::contact_element_id, 0);
    }
    auto view    = geometry::view(*slot);
    view.front() = id();

    return slot;
}

void to_json(Json& j, const ContactElement& element)
{
    j["id"]   = element.id();
    j["name"] = element.name();
}

void from_json(const Json& j, ContactElement& element)
{
    j["id"].get_to(element.m_id);
    j["name"].get_to(element.m_name);
}
}  // namespace uipc::core
