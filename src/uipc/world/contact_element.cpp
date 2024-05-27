#include <uipc/world/contact_element.h>

namespace uipc::world
{
ContactElement::ContactElement(I32 id, std::string_view name) noexcept
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
}  // namespace uipc::world
