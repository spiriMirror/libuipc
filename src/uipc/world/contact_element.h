#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/json.h>
#include <uipc/geometry/geometry.h>
namespace uipc::world
{
class ContactElement
{
  public:
    ContactElement() = default;
    ContactElement(IndexT id, std::string_view name) noexcept;
    IndexT           id() const noexcept;
    std::string_view name() const noexcept;

    friend void to_json(Json& j, const ContactElement& element);
    friend void from_json(const Json& j, ContactElement& element);

    void apply_to(geometry::Geometry& geo) const;

  private:
    IndexT      m_id = -1;
    std::string m_name;
};

void to_json(Json& j, const ContactElement& element);
void from_json(const Json& j, ContactElement& element);

}  // namespace uipc::world
