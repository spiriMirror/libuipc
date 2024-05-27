#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/json.h>
namespace uipc::world
{
class ContactElement
{
  public:
    IndexT           id() const noexcept;
    std::string_view name() const noexcept;

    friend void to_json(Json& j, const ContactElement& element);
    friend void from_json(const Json& j, ContactElement& element);

  private:
    friend class ContactTabular;
    ContactElement(IndexT id, std::string_view name) noexcept;
    IndexT      m_id;
    std::string m_name;
};

void to_json(Json& j, const ContactElement& element);
void from_json(const Json& j, ContactElement& element);

}  // namespace uipc::world
