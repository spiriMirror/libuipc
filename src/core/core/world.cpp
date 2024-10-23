#include <uipc/core/world.h>
#include <uipc/core/engine.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/sanity_check/sanity_checker_collection.h>
#include <uipc/builtin/attribute_name.h>
#include <magic_enum.hpp>
#include <uipc/common/zip.h>

namespace uipc::core
{
World::World(Engine& e) noexcept
    : m_engine(&e)
{
}

void World::init(Scene& s)
{
    // 1) Sanity check the scene
    sanity_check(s);

    if(!m_valid)
    {
        spdlog::error("World is not valid, skipping init.");
        return;
    }
    backend::WorldVisitor visitor{*this};
    m_scene = &s;
    m_scene->init(visitor);
    m_engine->init(visitor);
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

bool World::recover(SizeT aim_frame)
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
    return m_engine->recover(aim_frame);
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
}  // namespace uipc::core
