#include <uipc/core/subscene_model.h>
#include <uipc/core/subscene_tabular.h>
#include <uipc/common/log.h>
#include <algorithm>
#include <uipc/common/map.h>
#include <uipc/common/unit.h>
#include <uipc/geometry/attribute_collection.h>
#include <uipc/builtin/attribute_name.h>

namespace std
{
template <>
struct less<uipc::Vector2i>
{
    bool operator()(const uipc::Vector2i& L, const uipc::Vector2i& R) const
    {
        return L.x() < R.x() || (L.x() == R.x() && L.y() < R.y());
    }
};
}  // namespace std

namespace uipc::core
{
class SubsceneTabular::Impl
{
  public:
    Impl() noexcept
    {
        m_elements.reserve(64);

        // create the subscene models.
        m_topo       = m_models.create<Vector2i>("topo");
        m_is_enabled = m_models.create<IndexT>("is_enabled");

        // reserve the memory for subscene models.
        m_models.reserve(m_model_capacity);

        auto default_element = create("default");
        insert(default_element, default_element, true, default_config());
    }

    SubsceneElement create(std::string_view name) noexcept
    {
        auto   id = current_element_id();
        string name_str{name};
        if(name_str.empty())
            name_str = fmt::format("#{}", id);

        m_elements.push_back(SubsceneElement(id, name_str));

        return m_elements.back();
    }

    SubsceneElement default_element() const noexcept
    {
        return m_elements.front();
    }

    IndexT insert(const SubsceneElement& L, const SubsceneElement& R, bool enable, const Json& config)
    {
        Vector2i ids = {L.id(), R.id()};

        // check if the subscene element id is valid.
        UIPC_ASSERT(L.id() < current_element_id() && L.id() >= 0
                        && R.id() < current_element_id() && R.id() >= 0,
                    "Invalid subscene element id, id should be in [{},{}), your L={}, R={}.",
                    0,
                    current_element_id(),
                    L.id(),
                    R.id());

        // check if the name is matched.
        UIPC_ASSERT(m_elements[L.id()].name() == L.name()
                        && m_elements[R.id()].name() == R.name(),
                    "Subscene element name is not matched, L=<{},{}({} required)>, R=<{},{}({} required)>,"
                    "It seems the subscene element and subscene model don't come from the same SubsceneTabular.",
                    L.id(),
                    L.name(),
                    m_elements[L.id()].name(),
                    R.id(),
                    R.name(),
                    m_elements[R.id()].name());

        // ensure ids.x() < ids.y(), because the subscene model is symmetric.
        if(ids.x() > ids.y())
            std::swap(ids.x(), ids.y());

        auto it = m_model_map.find(ids);

        IndexT index;

        if(it != m_model_map.end())
        {
            // replace the existing one.
            index = it->second;
        }
        else
        {
            index            = m_models.size();
            m_model_map[ids] = index;

            _append_subscene_models();
        }

        view(*m_topo)[index]       = ids;
        view(*m_is_enabled)[index] = enable;

        return index;
    }

    IndexT current_element_id() const noexcept { return m_elements.size(); }

    IndexT index_at(IndexT i, IndexT j) const
    {
        Vector2i ids{i, j};
        if(ids.x() > ids.y())
            std::swap(ids.x(), ids.y());

        auto it = m_model_map.find(ids);
        return it != m_model_map.end() ? it->second : -1;
    }

    SubsceneModel at(IndexT i, IndexT j) const
    {
        auto idx    = index_at(i, j);
        bool enable = false;

        // According to Specification:
        // If set, use the stored value,
        // If not set, the mask matrix is Identity Matrix
        if(idx >= 0)
        {
            enable = m_is_enabled->view()[idx];
        }
        else
        {
            enable = (i == j);
        }
        return SubsceneModel{Vector2i{i, j}, enable, Json::object()};
    }

    const geometry::AttributeCollection& subscene_models() const noexcept
    {
        return m_models;
    }

    geometry::AttributeCollection& subscene_models() noexcept
    {
        return m_models;
    }

    SizeT element_count() const noexcept { return m_elements.size(); }

    static Json default_config() noexcept { return Json::object(); }

    vector<SubsceneElement>       m_elements;
    geometry::AttributeCollection m_models;
    SizeT                         m_model_capacity = 1024;

    mutable map<Vector2i, IndexT> m_model_map;

    mutable S<geometry::AttributeSlot<Vector2i>> m_topo;
    mutable S<geometry::AttributeSlot<IndexT>>   m_is_enabled;

    void _append_subscene_models()
    {
        auto new_size = m_models.size() + 1;
        if(m_model_capacity < new_size)
        {
            m_model_capacity *= 2;
            m_models.reserve(m_model_capacity);
        }
        m_models.resize(new_size);
    }

    void build_from(const geometry::AttributeCollection& ac, span<const SubsceneElement> ce)
    {
        m_elements.clear();
        m_elements = vector<SubsceneElement>(ce.begin(), ce.end());

        m_models = ac;
        m_topo   = m_models.find<Vector2i>("topo");
        UIPC_ASSERT(m_topo, "Subscene model topology is not found, please check the attribute collection.");
        m_is_enabled = m_models.find<IndexT>("is_enabled");
        UIPC_ASSERT(m_is_enabled, "Subscene model is_enabled is not found, please check the attribute collection.");

        m_model_map.clear();
        auto topo_view = m_topo->view();
        for(SizeT i = 0; i < topo_view.size(); ++i)
        {
            auto ids         = topo_view[i];
            m_model_map[ids] = i;
        }
    }

    void update_from(const geometry::AttributeCollectionCommit& ac,
                     span<const SubsceneElement>                ce)
    {
        m_elements.clear();
        m_elements = vector<SubsceneElement>(ce.begin(), ce.end());

        m_models.update_from(ac);
        m_topo = m_models.find<Vector2i>("topo");
        UIPC_ASSERT(m_topo, "Subscene model topology is not found, please check the attribute collection.");
        m_is_enabled = m_models.find<IndexT>("is_enabled");
        UIPC_ASSERT(m_is_enabled, "Subscene model is_enabled is not found, please check the attribute collection.");

        auto topo_view = m_topo->view();
        for(SizeT i = 0; i < topo_view.size(); ++i)
        {
            auto ids         = topo_view[i];
            m_model_map[ids] = i;
        }
    }
};

SubsceneTabular::SubsceneTabular() noexcept
    : m_impl(uipc::make_unique<Impl>())
{
}

SubsceneTabular::~SubsceneTabular() noexcept {}

SubsceneElement SubsceneTabular::create(std::string_view name) noexcept
{
    return m_impl->create(name);
}

SubsceneElement SubsceneTabular::default_element() const noexcept
{
    return m_impl->default_element();
}

IndexT SubsceneTabular::insert(const SubsceneElement& L,
                               const SubsceneElement& R,
                               bool                   enable,
                               const Json&            config)
{
    return m_impl->insert(L, R, enable, config);
}

SubsceneModel SubsceneTabular::at(IndexT i, IndexT j) const
{
    return m_impl->at(i, j);
}

SubsceneModelCollection SubsceneTabular::subscene_models() noexcept
{
    return m_impl->subscene_models();
}

CSubsceneModelCollection SubsceneTabular::subscene_models() const noexcept
{
    return m_impl->subscene_models();
}

geometry::AttributeCollection& SubsceneTabular::internal_subscene_models() const noexcept
{
    return m_impl->subscene_models();
}

span<SubsceneElement> SubsceneTabular::subscene_elements() const noexcept
{
    return m_impl->m_elements;
}

SizeT SubsceneTabular::element_count() const noexcept
{
    return m_impl->element_count();
}

Json SubsceneTabular::default_config() noexcept
{
    return Impl::default_config();
}

void SubsceneTabular::build_from(const geometry::AttributeCollection& ac,
                                 span<const SubsceneElement>          ce)
{
    m_impl->build_from(ac, ce);
}

void SubsceneTabular::update_from(const geometry::AttributeCollectionCommit& ac,
                                  span<const SubsceneElement>                ce)
{
    m_impl->update_from(ac, ce);
}

void to_json(Json& j, const SubsceneTabular& ct)
{
    j["subscene_elements"] = ct.subscene_elements();
    j["subscene_models"]   = ct.subscene_models().to_json();
}
}  // namespace uipc::core
