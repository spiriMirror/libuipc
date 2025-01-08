#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/common/type_define.h>
#include <uipc/common/json.h>

namespace uipc::core
{
class UIPC_CORE_API ContactModel
{
  public:
    ContactModel() noexcept;
    ContactModel(const Vector2i& ids, Float friction_rate, Float resistance, bool enable, const Json& config);

    const Vector2i& topo() const;
    Float           friction_rate() const;
    Float           resistance() const;
    bool            is_enabled() const;
    const Json&     config() const;

    friend void to_json(Json& json, const ContactModel& model);
    friend void from_json(const Json& json, ContactModel& model);

  private:
    Vector2i m_ids;
    Float    m_friction_rate;
    Float    m_resistance;
    bool     m_enabled;
    Json     m_config;
};

void to_json(Json& json, const ContactModel& model);

void from_json(const Json& json, ContactModel& model);
}  // namespace uipc::core
