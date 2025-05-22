#pragma once
#include <uipc/geometry/attribute_slot.h>

namespace uipc::geometry
{
class UIPC_CORE_API AttributeFactory
{
    class Impl;

  public:
    AttributeFactory();
    ~AttributeFactory();

    [[nodiscard]] vector<S<IAttributeSlot>> from_json(const Json& j);
    [[nodiscard]] Json to_json(span<IAttribute*> attributes);

  private:
    U<Impl> m_impl;
};
}  // namespace uipc::geometry
