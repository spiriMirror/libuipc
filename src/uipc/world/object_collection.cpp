#include <uipc/world/object_collection.h>
#include <uipc/common/log.h>
namespace uipc::world
{
template <typename T>
T& remove_const(const T& t)
{
    return const_cast<T&>(t);
}

P<ObjectSlot> ObjectCollection::emplace(Object&& object)
{
    IndexT id  = m_current_id++;
    auto   ptr = std::make_shared<ObjectSlot>(id, std::move(object));
    m_objects.emplace(id, ptr);
    return ptr;
}

P<ObjectSlot> ObjectCollection::pending_emplace(Object&& object)
{
    IndexT id  = m_current_id++;
    auto   ptr = std::make_shared<ObjectSlot>(id, std::move(object));
    m_pending_create.emplace(id, ptr);
    return ptr;
}

P<ObjectSlot> ObjectCollection::find(IndexT id) noexcept
{
    if(auto it = m_pending_destroy.find(id); it != m_pending_destroy.end())
        return {};

    if(auto it = m_pending_create.find(id); it != m_objects.end())
        return it->second;

    if(auto it = m_objects.find(id); it != m_objects.end())
        return it->second;

    return {};
}

P<const ObjectSlot> ObjectCollection::find(IndexT id) const noexcept
{
    return remove_const(*this).find(id);
}

void ObjectCollection::destroy(IndexT id) noexcept
{
    auto it = m_objects.find(id);
    if(it != m_objects.end())
        m_objects.erase(it);
    else
        UIPC_WARN_WITH_LOCATION("Try to destroy object({}) that does not exist, ignored.", id);
}

void ObjectCollection::pending_destroy(IndexT id) noexcept
{
    if(m_objects.find(id) != m_objects.end())
        m_pending_destroy.insert(id);
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

void ObjectCollection::solve_pending() noexcept
{
    for(auto& [id, slot] : m_pending_create)
    {
        UIPC_ASSERT(m_objects.find(id) == m_objects.end(),
                    "Object({}) already exists. Why can this happen?",
                    id);
        m_objects.emplace(id, slot);
    }
    m_pending_create.clear();

    for(auto id : m_pending_destroy)
    {
        UIPC_ASSERT(m_objects.find(id) != m_objects.end(),
                    "Object({}) does not exist. Why can this happen?",
                    id);
        m_objects.erase(id);
    }
    m_pending_destroy.clear();
}
}  // namespace uipc::world
