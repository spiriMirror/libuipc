#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/attribute_slot.h>

namespace uipc::geometry
{
class UIPC_IO_API AttributeIO final
{
  public:
    class Interface;

    AttributeIO(std::string_view file);
    ~AttributeIO() noexcept;

    void read(std::string_view name, IAttributeSlot& slot);

  private:
    U<Interface> m_impl;
};

class UIPC_IO_API AttributeIOError : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::geometry
