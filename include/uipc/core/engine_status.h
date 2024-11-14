#pragma once
#include <uipc/common/dllexport.h>
#include <string>
#include <uipc/common/json.h>
#include <uipc/common/smart_pointer.h>

namespace uipc::core
{
class UIPC_CORE_API EngineStatus
{
  public:
    enum class Type : int
    {
        None,
        Info,
        Warning,
        Error,
    };

    EngineStatus()  = default;
    ~EngineStatus() = default;

    Type             type() const noexcept;
    std::string_view what() const noexcept;

    static EngineStatus info(std::string_view msg);
    static EngineStatus warning(std::string_view msg);
    static EngineStatus error(std::string_view msg);

  private:
    Type        m_type = Type::None;
    std::string m_msg;
};

class UIPC_CORE_API EngineStatusCollection
{
  public:
    EngineStatusCollection();
    ~EngineStatusCollection();

    void push_back(const EngineStatus& error);
    void push_back(EngineStatus&& error);

    void clear();

    bool has_error() const noexcept;

    Json to_json() const;

  private:
    class Impl;
    U<Impl> m_impl;
};
}  // namespace uipc::core
