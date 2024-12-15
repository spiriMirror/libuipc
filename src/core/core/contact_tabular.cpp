#include <uipc/core/contact_tabular.h>
#include <uipc/common/log.h>
#include <algorithm>
#include <uipc/common/map.h>
#include <uipc/common/unit.h>

namespace std
{
bool operator<(const uipc::Vector2i& L, const uipc::Vector2i& R)
{
    return L.x() < R.x() || (L.x() == R.x() && L.y() < R.y());
}
}  // namespace std

namespace uipc::core
{
class ContactTabular::Impl
{
  public:
    Impl() noexcept
        : m_current_id{0}
    {
        m_models.reserve(64);
        m_elements.reserve(64);

        auto default_element = create("default");
        insert(default_element, default_element, 0.5, 1.0_GPa, true, default_config());
    }

    ContactElement& create(std::string_view name) noexcept
    {
        string name_str{name};
        if(name_str.empty())
            name_str = fmt::format("_{}", m_current_id);
        m_elements.push_back(uipc::make_unique<ContactElement>(m_current_id++, name_str));
        return *m_elements.back();
    }

    void insert(const ContactElement& L,
                const ContactElement& R,
                Float                 friction_rate,
                Float                 resistance,
                bool                  enable,
                const Json&           config)
    {
        m_is_dirty = true;

        auto     model_id = m_models.size();
        Vector2i ids      = {L.id(), R.id()};

        // check if the contact element id is valid.
        UIPC_ASSERT(L.id() < m_current_id && L.id() >= 0
                        && R.id() < m_current_id && R.id() >= 0,
                    "Invalid contact element id, id should be in [{},{}), your L={}, R={}.",
                    0,
                    m_current_id,
                    L.id(),
                    R.id());

        // check if the name is matched.
        UIPC_ASSERT(m_elements[L.id()]->name() == L.name()
                        && m_elements[R.id()]->name() == R.name(),
                    "Contact element name is not matched, L=<{},{}({} required)>, R=<{},{}({} required)>,"
                    "It seems the contact element and contact model don't come from the same ContactTabular.",
                    L.id(),
                    L.name(),
                    m_elements[L.id()]->name(),
                    R.id(),
                    R.name(),
                    m_elements[R.id()]->name());

        // ensure ids.x() < ids.y(), because the contact model is symmetric.
        if(ids.x() > ids.y())
            std::swap(ids.x(), ids.y());

        auto it = m_model_map.find(ids);

        if(it != m_model_map.end())
        {
            it->second = ContactModel{ids, friction_rate, resistance, enable, config};
            UIPC_WARN_WITH_LOCATION("Contact model between {}[{}] and {}[{}] already exists, replace the old one.",
                                    m_elements[L.id()]->name(),
                                    L.id(),
                                    m_elements[R.id()]->name(),
                                    R.id());
        }
        else
        {
            m_model_map[ids] =
                ContactModel{ids, friction_rate, resistance, enable, config};
        }
    }


    void default_model(Float friction_rate, Float resistance, const Json& config) noexcept
    {
        m_model_map[Vector2i::Zero()] =
            ContactModel{Vector2i::Zero(), friction_rate, resistance, true, config};
    }

    ContactElement& default_element() noexcept { return *m_elements.front(); }

    const ContactModel& default_model() const noexcept
    {
        return m_model_map.at(Vector2i::Zero());
    }

    std::span<const ContactModel> contact_models() const noexcept
    {
        if(m_is_dirty)
        {
            m_is_dirty = false;
            m_models.resize(m_model_map.size());
            std::transform(m_model_map.begin(),
                           m_model_map.end(),
                           m_models.begin(),
                           [](const auto& p) { return p.second; });
        }
        return m_models;
    }

    SizeT element_count() const noexcept { return m_elements.size(); }

    static Json default_config() noexcept { return Json::object(); }

    IndexT                       m_current_id;
    mutable bool                 m_is_dirty = true;
    mutable vector<ContactModel> m_models;
    vector<U<ContactElement>>    m_elements;
    map<Vector2i, ContactModel>  m_model_map;
};

ContactTabular::ContactTabular() noexcept
    : m_impl(uipc::make_unique<Impl>())
{
}

ContactTabular::~ContactTabular() noexcept {}

ContactElement& ContactTabular::create(std::string_view name) noexcept
{
    return m_impl->create(name);
}

void ContactTabular::insert(const ContactElement& L,
                            const ContactElement& R,
                            Float                 friction_rate,
                            Float                 resistance,
                            bool                  enable,
                            const Json&           config)
{
    m_impl->insert(L, R, friction_rate, resistance, enable, config);
}

void ContactTabular::default_model(Float friction_rate, Float resistance, const Json& config) noexcept
{
    m_impl->default_model(friction_rate, resistance, config);
}

ContactElement& ContactTabular::default_element() noexcept
{
    return m_impl->default_element();
}

const ContactModel& ContactTabular::default_model() const noexcept
{
    return m_impl->default_model();
}

std::span<const ContactModel> ContactTabular::contact_models() const noexcept
{
    return m_impl->contact_models();
}

SizeT ContactTabular::element_count() const noexcept
{
    return m_impl->element_count();
}

Json ContactTabular::default_config() noexcept
{
    return Impl::default_config();
}

void to_json(Json& j, const ContactTabular& ct)
{
    std::vector<ContactElement> elements(ct.element_count());

    auto& element_ptrs = ct.m_impl->m_elements;

    std::ranges::transform(
        element_ptrs, elements.begin(), [](const auto& e) { return *e; });

    j["elements"] = elements;
    j["models"]   = ct.contact_models();
}
}  // namespace uipc::core
