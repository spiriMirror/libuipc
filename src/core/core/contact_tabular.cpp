#include <uipc/core/contact_model.h>
#include <uipc/core/contact_tabular.h>
#include <uipc/common/log.h>
#include <algorithm>
#include <uipc/common/map.h>
#include <uipc/common/unit.h>
#include <uipc/geometry/attribute_collection.h>
#include <uipc/builtin/attribute_name.h>

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
    {
        m_elements.reserve(64);

        // create the contact models.
        m_topo           = m_contact_models.create<Vector2i>("topo");
        m_friction_rates = m_contact_models.create<Float>("friction_rate");
        m_resistances    = m_contact_models.create<Float>("resistance");
        m_is_enabled     = m_contact_models.create<IndexT>("is_enabled");

        // reserve the memory for contact models.
        m_contact_models.reserve(m_contact_model_capacity);


        auto default_element = create("default");
        insert(default_element, default_element, 0.5, 1.0_GPa, true, default_config());
    }

    ContactElement& create(std::string_view name) noexcept
    {
        string name_str{name};
        if(name_str.empty())
            name_str = fmt::format("_{}", m_current_element_id);
        m_elements.push_back(
            uipc::make_unique<ContactElement>(m_current_element_id++, name_str));
        return *m_elements.back();
    }

    IndexT insert(const ContactElement& L,
                  const ContactElement& R,
                  Float                 friction_rate,
                  Float                 resistance,
                  bool                  enable,
                  const Json&           config)
    {
        Vector2i ids = {L.id(), R.id()};

        // check if the contact element id is valid.
        UIPC_ASSERT(L.id() < m_current_element_id && L.id() >= 0
                        && R.id() < m_current_element_id && R.id() >= 0,
                    "Invalid contact element id, id should be in [{},{}), your L={}, R={}.",
                    0,
                    m_current_element_id,
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

        IndexT index;

        if(it != m_model_map.end())
        {
            index = it->second;
            UIPC_WARN_WITH_LOCATION("Contact model between {}[{}] and {}[{}] already exists, replace the old one.",
                                    m_elements[L.id()]->name(),
                                    L.id(),
                                    m_elements[R.id()]->name(),
                                    R.id());
        }
        else
        {
            index            = m_contact_models.size();
            m_model_map[ids] = index;

            _append_contact_models();
        }

        view(*m_topo)[index]           = ids;
        view(*m_friction_rates)[index] = friction_rate;
        view(*m_resistances)[index]    = resistance;
        view(*m_is_enabled)[index]     = enable;

        return index;
    }

    IndexT index_at(SizeT i, SizeT j) const
    {
        Vector2i ids{i, j};
        if(ids.x() > ids.y())
            std::swap(ids.x(), ids.y());

        auto it = m_model_map.find(ids);
        return it != m_model_map.end() ? it->second : 0;
    }

    ContactModel at(SizeT i, SizeT j) const
    {
        return ContactModel{Vector2i{i, j},
                            view(*m_friction_rates)[index_at(i, j)],
                            view(*m_resistances)[index_at(i, j)],
                            view(*m_is_enabled)[index_at(i, j)] != 0,
                            Json::object()};
    }

    void default_model(Float friction_rate, Float resistance, bool enable, const Json& config) noexcept
    {
        view(*m_friction_rates)[0] = friction_rate;
        view(*m_resistances)[0]    = resistance;
        view(*m_is_enabled)[0]     = enable;
    }

    ContactModel default_model() const noexcept { return at(0, 0); }

    ContactElement& default_element() noexcept { return *m_elements.front(); }

    const geometry::AttributeCollection& contact_models() const noexcept
    {
        return m_contact_models;
    }

    geometry::AttributeCollection& contact_models() noexcept
    {
        return m_contact_models;
    }

    SizeT element_count() const noexcept { return m_elements.size(); }

    static Json default_config() noexcept { return Json::object(); }

    IndexT m_current_element_id = 0;
    // mutable bool m_is_dirty = true;
    // mutable vector<ContactModel> m_models;
    vector<U<ContactElement>> m_elements;

    map<Vector2i, IndexT> m_model_map;

    geometry::AttributeCollection m_contact_models;
    SizeT                         m_contact_model_capacity = 1024;
    mutable S<geometry::AttributeSlot<Vector2i>> m_topo;
    mutable S<geometry::AttributeSlot<Float>>    m_friction_rates;
    mutable S<geometry::AttributeSlot<Float>>    m_resistances;
    mutable S<geometry::AttributeSlot<IndexT>>   m_is_enabled;

    void _append_contact_models()
    {
        auto new_size = m_contact_models.size() + 1;
        if(m_contact_model_capacity < new_size)
        {
            m_contact_model_capacity *= 2;
            m_contact_models.reserve(m_contact_model_capacity);
        }
        m_contact_models.resize(new_size);
    }
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

IndexT ContactTabular::insert(const ContactElement& L,
                              const ContactElement& R,
                              Float                 friction_rate,
                              Float                 resistance,
                              bool                  enable,
                              const Json&           config)
{
    return m_impl->insert(L, R, friction_rate, resistance, enable, config);
}

ContactModel ContactTabular::at(SizeT i, SizeT j) const
{
    return m_impl->at(i, j);
}

void ContactTabular::default_model(Float       friction_rate,
                                   Float       resistance,
                                   bool        enable,
                                   const Json& config) noexcept
{
    m_impl->default_model(friction_rate, resistance, enable, config);
}

ContactElement& ContactTabular::default_element() noexcept
{
    return m_impl->default_element();
}

ContactModel ContactTabular::default_model() const noexcept
{
    return m_impl->default_model();
}

ContactModelCollection ContactTabular::contact_models() noexcept
{
    return m_impl->contact_models();
}

CContactModelCollection ContactTabular::contact_models() const noexcept
{
    return m_impl->contact_models();
}

geometry::AttributeCollection& ContactTabular::internal_contact_models() noexcept
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
    j["models"]   = ct.contact_models().to_json();
}
}  // namespace uipc::core
