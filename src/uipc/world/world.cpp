#include <uipc/world/world.h>
#include <uipc/backend/visitors/world_visitor.h>

namespace uipc ::world
{
World::World(engine::IEngine& e) noexcept
    : m_engine(&e)
{
}

void World::init(Scene& s)
{
    m_scene = &s;
    m_engine->init(backend::WorldVisitor{*this});
}

void World::advance() {}

void World::sync() {}

void World::retrieve() {}
}  // namespace uipc::world
