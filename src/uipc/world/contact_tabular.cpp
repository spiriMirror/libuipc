#include <uipc/world/contact_tabular.h>
#include <uipc/common/log.h>
#include <algorithm>

namespace uipc::world
{
ContactTabular::ContactTabular() noexcept
    : m_current_id{0}
{
    m_models.reserve(64);
    m_elements.reserve(64);

    auto default_element = create("default");
    insert(default_element, default_element, 0.5, 1e8, {});
}

ContactElement ContactTabular::create(std::string_view name) noexcept
{
    std::string name_str{name};
    if(name_str.empty())
        name_str = fmt::format("_{}", m_current_id);
    m_elements.push_back({m_current_id++, name_str});
    return m_elements.back();
}

void ContactTabular::insert(const ContactElement& L,
                            const ContactElement& R,
                            Float                 friction_rate,
                            Float                 resistance,
                            const Json&           config)
{
    auto     model_id = m_models.size();
    Vector2i ids      = {L.id(), R.id()};

    // check if the contact element id is valid.
    UIPC_ASSERT(L.id() < m_current_id && L.id() >= 0 && R.id() < m_current_id && R.id() >= 0,
                "Invalid contact element id, id should be in [{},{}), your L={}, R={}.",
                0,
                m_current_id,
                L.id(),
                R.id());

    // check if the name is matched.
    UIPC_ASSERT(m_elements[L.id()].name() == L.name()
                    && m_elements[R.id()].name() == R.name(),
                "Contact element name is not matched, L=<{},{}({} required)>, R=<{},{}({} required)>,"
                "It seems the contact element and contact model don't come from the same ContactTabular.",
                L.id(),
                L.name(),
                m_elements[L.id()].name(),
                R.id(),
                R.name(),
                m_elements[R.id()].name());

    // ensure ids.x() < ids.y(), because the contact model is symmetric.
    if(ids.x() > ids.y())
        std::swap(ids.x(), ids.y());

    auto pred = [](const Vector2i& R, const ContactModel& L) -> bool
    {
        return L.ids().x() > R.x() || (L.ids().x() == R.x() && L.ids().y() > R.y());
    };

    // find the place to insert the new contact model.
    auto insert_place = std::upper_bound(m_models.begin(), m_models.end(), ids, pred);

    UIPC_ASSERT(insert_place == m_models.end() || insert_place->ids() != ids,
                "Contact model between {}[{}] and {}[{}] already exists.",
                m_elements[L.id()].name(),
                L.id(),
                m_elements[R.id()].name(),
                R.id());

    if(insert_place == m_models.end() || insert_place == m_models.begin())
    {
        // insert the new contact model.
        m_models.emplace(insert_place, ids, friction_rate, resistance, config);
    }
    else
    {
        auto prev = insert_place - 1;
        if(prev->ids() == ids)
        {
            *prev = ContactModel{ids, friction_rate, resistance, config};
            UIPC_WARN_WITH_LOCATION(
                "Contact model between {}[{}] and {}[{}] already exists, "
                "replace the old one.",
                m_elements[L.id()].name(),
                L.id(),
                m_elements[R.id()].name(),
                R.id());
        }
        else
        {
            // insert the new contact model.
            m_models.emplace(insert_place, ids, friction_rate, resistance, config);
        }
    }
}

void ContactTabular::default_model(Float friction_rate, Float resistance, const Json& config) noexcept
{
    m_models.front() = ContactModel{Vector2i::Zero(), friction_rate, resistance, config};
}

ContactElement ContactTabular::default_element() noexcept
{
    return m_elements.front();
}

const ContactModel& ContactTabular::default_model() const noexcept
{
    return m_models.front();
}

std::span<const ContactModel> ContactTabular::contact_models() const noexcept
{
    return m_models;
}

void to_json(Json& j, const ContactTabular& ct)
{
    j["elements"] = ct.m_elements;
    j["models"]   = ct.m_models;
}
}  // namespace uipc::world
