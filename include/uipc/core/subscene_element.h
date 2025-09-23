#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/string.h>
#include <uipc/common/json.h>
#include <uipc/geometry/geometry.h>
namespace uipc::core
{
class UIPC_CORE_API SubsceneElement
{
  public:
    SubsceneElement() = default;

    SubsceneElement(const SubsceneElement&)            = default;
    SubsceneElement(SubsceneElement&&)                 = default;
    SubsceneElement& operator=(const SubsceneElement&) = default;
    SubsceneElement& operator=(SubsceneElement&&)      = default;

    SubsceneElement(IndexT id, std::string_view name) noexcept;
    IndexT           id() const noexcept;
    std::string_view name() const noexcept;

    friend void to_json(Json& j, const SubsceneElement& element);
    friend void from_json(const Json& j, SubsceneElement& element);

    S<geometry::AttributeSlot<IndexT>> apply_to(geometry::Geometry& geo) const;

  private:
    IndexT m_id = -1;
    string m_name;
};

void to_json(Json& j, const SubsceneElement& element);
void from_json(const Json& j, SubsceneElement& element);
}  // namespace uipc::core
