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
    m_scene->m_started = true;
}

void World::advance()
{
    m_engine->advance();
}

void World::sync()
{
    m_engine->sync();
}

void World::retrieve()
{
    m_engine->retrieve();
}
}  // namespace uipc::world
