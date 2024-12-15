#include <uipc/common/uipc.h>
#include <filesystem>

namespace uipc
{
namespace fs = std::filesystem;

static Json m_config = default_config();

Json default_config()
{
    Json j = Json::object();
    // j["version"]    = "1.0.0";
    j["module_dir"] = "";
    return j;
}

const Json& config()
{
    return m_config;
}

void init(const Json& config)
{
    uipc::m_config = config;

    if(m_config.find("module_dir") == m_config.end())
    {
        throw std::runtime_error("module_dir is not set in config.");
    }
    else
    {
        auto module_dir = m_config["module_dir"].get<std::string>();
        if(!fs::exists(module_dir))
        {
            throw std::runtime_error("module_dir does not exist.");
        }
    }
}
}  // namespace uipc
