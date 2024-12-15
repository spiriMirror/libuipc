#include <uipc/core/engine_status.h>
#include <uipc/common/list.h>
#include <uipc/common/smart_pointer.h>

namespace uipc::core
{
auto EngineStatus::type() const noexcept -> Type
{
    return m_type;
}

std::string_view EngineStatus::what() const noexcept
{
    return m_msg;
}

EngineStatus EngineStatus::info(std::string_view msg)
{
    EngineStatus status;
    status.m_type = Type::Info;
    status.m_msg  = msg;
    return status;
}

EngineStatus EngineStatus::warning(std::string_view msg)
{
    EngineStatus status;
    status.m_type = Type::Warning;
    status.m_msg  = msg;
    return status;
}

EngineStatus EngineStatus::error(std::string_view msg)
{
    EngineStatus status;
    status.m_type = Type::Error;
    status.m_msg  = msg;
    return status;
}

class EngineStatusCollection::Impl
{
  public:
    Impl()  = default;
    ~Impl() = default;

    void push_back(const EngineStatus& status)
    {
        switch(status.type())
        {
            case EngineStatus::Type::Info:
                m_infos.push_back(uipc::make_unique<EngineStatus>(status));
                break;
            case EngineStatus::Type::Warning:
                m_warnings.push_back(uipc::make_unique<EngineStatus>(status));
                break;
            case EngineStatus::Type::Error:
                m_errors.push_back(uipc::make_unique<EngineStatus>(status));
                break;
            default:
                break;
        }
    }

    void push_back(EngineStatus&& status)
    {
        switch(status.type())
        {
            case EngineStatus::Type::Info:
                m_infos.push_back(uipc::make_unique<EngineStatus>(std::move(status)));
                break;
            case EngineStatus::Type::Warning:
                m_warnings.push_back(uipc::make_unique<EngineStatus>(std::move(status)));
                break;
            case EngineStatus::Type::Error:
                m_errors.push_back(uipc::make_unique<EngineStatus>(std::move(status)));
                break;
            default:
                break;
        }
    }

    void clear()
    {
        m_infos.clear();
        m_warnings.clear();
        m_errors.clear();
    }

    bool has_error() const noexcept { return !m_errors.empty(); }

    Json to_json() const
    {
        Json j     = Json::object();
        j["infos"] = Json::array();
        for(const auto& info : m_infos)
        {
            j["infos"].push_back(info->what());
        }

        j["warnings"] = Json::array();
        for(const auto& warning : m_warnings)
        {
            j["warnings"].push_back(warning->what());
        }

        j["errors"] = Json::array();
        for(const auto& error : m_errors)
        {
            j["errors"].push_back(error->what());
        }

        return j;
    }

  private:
    list<U<EngineStatus>> m_infos;
    list<U<EngineStatus>> m_warnings;
    list<U<EngineStatus>> m_errors;
};


EngineStatusCollection::EngineStatusCollection()
    : m_impl(uipc::make_unique<Impl>())

{
}

EngineStatusCollection::~EngineStatusCollection() {}

void EngineStatusCollection::push_back(const EngineStatus& error)
{
    m_impl->push_back(error);
}

void EngineStatusCollection::push_back(EngineStatus&& error)
{
    m_impl->push_back(std::move(error));
}

void EngineStatusCollection::clear()
{
    m_impl->clear();
}

bool EngineStatusCollection::has_error() const noexcept
{
    return m_impl->has_error();
}

Json EngineStatusCollection::to_json() const
{
    return m_impl->to_json();
}
}  // namespace uipc::core
