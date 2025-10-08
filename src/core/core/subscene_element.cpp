#include <uipc/core/subscene_element.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::core
{
SubsceneElement::SubsceneElement(IndexT id, std::string_view name) noexcept
    : m_id{id}
    , m_name{name}
{
}

IndexT SubsceneElement::id() const noexcept
{
    return m_id;
}

std::string_view SubsceneElement::name() const noexcept
{
    return m_name;
}

S<geometry::AttributeSlot<IndexT>> SubsceneElement::apply_to(geometry::Geometry& geo) const
{
    auto slot = geo.meta().find<IndexT>(builtin::subscene_element_id);
    if(!slot)
    {
        slot = geo.meta().create<IndexT>(builtin::subscene_element_id, 0);
    }
    auto view    = geometry::view(*slot);
    view.front() = id();

    return slot;
}

void to_json(Json& j, const SubsceneElement& element)
{
    j["id"]   = element.id();
    j["name"] = element.name();
}

void from_json(const Json& j, SubsceneElement& element)
{
    j["id"].get_to(element.m_id);
    j["name"].get_to(element.m_name);
}
}  // namespace uipc::core
