#include <uipc/core/contact_model.h>
#include <uipc/common/json_eigen.h>

namespace uipc::core
{
ContactModel::ContactModel() noexcept
    : m_ids(-1, -1)
    , m_friction_rate(0.0)
    , m_resistance(0.0)
    , m_enabled(false)
    , m_config(Json::object())
{
}

ContactModel::ContactModel(const Vector2i& ids,
                           Float           friction_rate,
                           Float           resistance,
                           bool            enable,
                           const Json&     config)
    : m_ids(ids.minCoeff(), ids.maxCoeff())
    , m_friction_rate(friction_rate)
    , m_resistance(resistance)
    , m_enabled(enable)
    , m_config(config)
{
}

const Vector2i& ContactModel::topo() const
{
    return m_ids;
}

Float ContactModel::friction_rate() const
{
    return m_friction_rate;
}

Float ContactModel::resistance() const
{
    return m_resistance;
}

bool ContactModel::is_enabled() const
{
    return m_enabled;
}

const Json& ContactModel::config() const
{
    return m_config;
}
void to_json(Json& json, const ContactModel& model)
{
    json = Json{{"ids", model.topo()},
                {"friction_rate", model.friction_rate()},
                {"resistance", model.resistance()},
                {"enabled", model.is_enabled()},
                {"config", model.config()}};
}

void from_json(const Json& json, ContactModel& model)
{
    model.m_ids           = json.at("ids").get<Vector2i>();
    model.m_friction_rate = json.at("friction_rate").get<Float>();
    model.m_resistance    = json.at("resistance").get<Float>();
    model.m_enabled       = json.at("enabled").get<bool>();
    model.m_config        = json.at("config");
}
}  // namespace uipc::core
