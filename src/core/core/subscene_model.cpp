#include <uipc/core/subscene_model.h>
#include <uipc/common/json_eigen.h>

namespace uipc::core
{
SubsceneModel::SubsceneModel() noexcept
    : m_ids(-1, -1)
    , m_is_enabled(false)
    , m_config(Json::object())
{
}

SubsceneModel::SubsceneModel(const Vector2i& ids, bool enable, const Json& config)
    : m_ids(ids.minCoeff(), ids.maxCoeff())
    , m_is_enabled(enable)
    , m_config(config)
{
}

const Vector2i& SubsceneModel::topo() const
{
    return m_ids;
}

bool SubsceneModel::is_enabled() const
{
    return m_is_enabled;
}

const Json& SubsceneModel::config() const
{
    return m_config;
}

void to_json(Json& json, const SubsceneModel& model)
{
    json = Json{{"ids", model.topo()},
                {"enabled", model.is_enabled()},
                {"config", model.config()}};
}

void from_json(const Json& json, SubsceneModel& model)
{
    model.m_ids        = json.at("ids").get<Vector2i>();
    model.m_is_enabled = json.at("enabled").get<bool>();
    model.m_config     = json.at("config");
}
}  // namespace uipc::core
