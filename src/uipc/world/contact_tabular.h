#pragma once
#include <uipc/world/contact_element.h>
#include <uipc/world/contact_model.h>
#include <uipc/common/json.h>
#include <uipc/common/unordered_map.h>
#include <uipc/common/vector.h>
#include <uipc/common/format.h>

namespace uipc::world
{
class ContactTabular
{
  public:
    ContactTabular() noexcept;

    ContactElement& create(std::string_view name = "") noexcept;

    void insert(const ContactElement& L,
                const ContactElement& R,
                Float                 friction_rate,
                Float                 resistance,
                const Json&           config = {});

    void default_model(Float friction_rate, Float resistance, const Json& config = {}) noexcept;
    ContactElement& default_element() noexcept;

    const ContactModel& default_model() const noexcept;

    std::span<const ContactModel> contact_models() const noexcept;

    friend void to_json(Json& j, const ContactTabular& ct);

  private:
    IndexT                       m_current_id;
    mutable vector<ContactModel> m_models;
    vector<U<ContactElement>>    m_elements;
};
}  // namespace uipc::world
