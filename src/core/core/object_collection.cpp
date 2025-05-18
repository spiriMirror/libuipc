#include <uipc/core/object_collection.h>
#include <uipc/common/log.h>
#include <uipc/core/object_snapshot.h>

namespace uipc::core
{
template <typename T>
T& remove_const(const T& t)
{
    return const_cast<T&>(t);
}

S<Object> ObjectCollection::emplace(Object&& object)
{
    IndexT id  = m_next_id++;
    auto   ptr = uipc::make_shared<Object>(std::move(object));
    m_objects.emplace(id, ptr);
    return ptr;
}

S<Object> ObjectCollection::find(IndexT id) noexcept
{
    if(auto it = m_objects.find(id); it != m_objects.end())
        return it->second;

    return {};
}

S<const Object> ObjectCollection::find(IndexT id) const noexcept
{
    return remove_const(*this).find(id);
}

vector<S<Object>> ObjectCollection::find(std::string_view name) noexcept
{
    vector<S<Object>> result;
    for(auto& [id, object] : m_objects)
    {
        if(object->name() == name)
            result.push_back(object);
    }
    return result;
}

vector<S<const Object>> ObjectCollection::find(std::string_view name) const noexcept
{
    vector<S<const Object>> result;
    for(auto& [id, object] : m_objects)
    {
        if(object->name() == name)
            result.push_back(object);
    }
    return result;
}

void ObjectCollection::destroy(IndexT id) noexcept
{
    auto it = m_objects.find(id);
    if(it != m_objects.end())
        m_objects.erase(it);
    else
        UIPC_WARN_WITH_LOCATION("Try to destroy object({}) that does not exist, ignored.", id);
}

void ObjectCollection::reserve(SizeT size) noexcept
{
    m_objects.reserve(size);
}

SizeT ObjectCollection::size() const noexcept
{
    return m_objects.size();
}
IndexT ObjectCollection::next_id() const noexcept
{
    return m_next_id;
}

unordered_map<IndexT, S<Object>>& ObjectCollection::objects()
{
    return m_objects;
}
const unordered_map<IndexT, S<Object>>& ObjectCollection::objects() const
{
    return m_objects;
}

void ObjectCollection::build_from(span<S<Object>> objects) noexcept
{
    m_objects.clear();
    m_next_id = 0;
    for(auto& object : objects)
    {
        auto id = object->id();
        if(id >= m_next_id)
            m_next_id = id + 1;
        m_objects.emplace(id, object);
    }
}

void ObjectCollection::update_from(internal::Scene& scene,
                                   span<const ObjectSnapshot> snapshots) noexcept
{
    for(auto& snapshot : snapshots)
    {
        auto it = m_objects.find(snapshot.m_id);
        if(it != m_objects.end())
        {
            auto& object = *it->second;
            object.update_from(snapshot);
        }
        else
        {
            auto id = snapshot.m_id;
            auto new_object = uipc::make_shared<Object>(scene, id, snapshot.m_name);
            new_object->build_from(snapshot.m_geometries);
            m_objects.emplace(id, new_object);
            if(id >= m_next_id)
                m_next_id = id + 1;
        }
    }
}
}  // namespace uipc::core

namespace fmt
{
appender fmt::formatter<uipc::core::ObjectCollection>::format(const uipc::core::ObjectCollection& c,
                                                              format_context& ctx) const
{
    fmt::format_to(ctx.out(), "Objects({}):", c.size());

    for(auto& [id, object] : c.m_objects)
    {
        fmt::format_to(ctx.out(), "\n  [{}] {}", object->id(), object->name());
    }

    return ctx.out();
}
}  // namespace fmt
