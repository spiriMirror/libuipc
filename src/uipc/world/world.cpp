#include <uipc/world/world.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/world/sanity_check/sanity_checker_collection.h>
#include <magic_enum.hpp>
namespace uipc::world
{
World::World(engine::IEngine& e) noexcept
    : m_engine(&e)
    , m_animator(*this)
{
}

void World::init(Scene& s)
{
    sanity_check(s);

    if(!m_valid)
    {
        spdlog::error("World is not valid, skipping init.");
        return;
    }

    m_scene = &s;
    m_engine->init(backend::WorldVisitor{*this});
    m_scene->m_impl.started = true;
}

void World::advance()
{
    if(!m_valid)
    {
        spdlog::error("World is not valid, skipping advance.");
        return;
    }
    m_engine->advance();
}

void World::sync()
{
    if(!m_valid)
    {
        spdlog::error("World is not valid, skipping sync.");
        return;
    }
    m_engine->sync();
}

void World::retrieve()
{
    if(!m_valid)
    {
        spdlog::error("World is not valid, skipping retrieve.");
        return;
    }
    m_engine->retrieve();
}

bool World::dump()
{
    if(!m_valid)
    {
        spdlog::error("World is not valid, skipping dump.");
        return false;
    }
    return m_engine->dump();
}

bool World::recover()
{
    if(!m_scene)
    {
        spdlog::warn("Scene has not been set, skipping recover. Hint: you may call World::init() first.");
        return false;
    }
    if(!m_valid)
    {
        spdlog::error("World is not valid, skipping recover.");
        return false;
    }
    return m_engine->recover();
}

SizeT World::frame() const
{
    if(!m_valid)
    {
        spdlog::error("World is not valid, frame set to 0.");
        return 0;
    }
    return m_engine->frame();
}

Animator& World::animator()
{
    return m_animator;
}

void World::sanity_check(Scene& s)
{
    if(s.info()["sanity_check"]["enable"] == true)
    {
        SanityCheckerCollection sanity_checkers;
        sanity_checkers.init(s);

        auto result = sanity_checkers.check();
        switch(result)
        {
            case SanityCheckResult::Success:
                spdlog::info("Scene sanity check passed.");
                break;
            case SanityCheckResult::Warning:
                spdlog::warn("Scene sanity check passed with warnings.");
                break;
            case SanityCheckResult::Error:
                spdlog::error("Scene sanity check failed, we invalidate World.");
                m_valid = false;
                break;
            default:
                break;
        }
    }
}
}  // namespace uipc::world
