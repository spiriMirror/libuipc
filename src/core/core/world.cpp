#include <uipc/core/world.h>
#include <uipc/core/engine.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/core/sanity_checker.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/zip.h>
#include <uipc/common/uipc.h>
#include <dylib.hpp>
#include <uipc/backend/module_init_info.h>


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

    if(m_engine->status().has_error())
    {
        spdlog::error("Engine has error after init, world becomes invalid.");
        m_valid = false;
    }
}

void World::advance()
{
    if(!m_valid)
    {
        spdlog::error("World is not valid, skipping advance.");
        return;
    }

    m_engine->advance();

    if(m_engine->status().has_error())
    {
        spdlog::error("Engine has error after advance, world becomes invalid.");
        m_valid = false;
    }
}

void World::sync()
{
    if(!m_valid)
    {
        spdlog::error("World is not valid, skipping sync.");
        return;
    }

    m_engine->sync();

    if(m_engine->status().has_error())
    {
        spdlog::error("Engine has error after sync, world becomes invalid.");
        m_valid = false;
    }
}

void World::retrieve()
{
    if(!m_valid)
    {
        spdlog::error("World is not valid, skipping retrieve.");
        return;
    }
    m_engine->retrieve();

    if(m_engine->status().has_error())
    {
        spdlog::error("Engine has error after retrieve, world becomes invalid.");
        m_valid = false;
    }
}

void World::backward()
{
    if(!m_valid)
    {
        spdlog::error("World is not valid, skipping backward.");
        return;
    }
    m_engine->backward();

    if(m_engine->status().has_error())
    {
        spdlog::error("Engine has error after backward, world becomes invalid.");
        m_valid = false;
    }
}

bool World::dump()
{
    if(!m_valid)
    {
        spdlog::error("World is not valid, skipping dump.");
        return false;
    }

    bool success   = m_engine->dump();
    bool has_error = m_engine->status().has_error();
    if(has_error)
    {
        spdlog::error("Engine has error after dump, world becomes invalid.");
        m_valid = false;
    }

    return success && !has_error;
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
    bool success   = m_engine->recover(aim_frame);
    bool has_error = m_engine->status().has_error();

    if(has_error)
    {
        spdlog::error("Engine has error after recover, world becomes invalid.");
        m_valid = false;
    }

    return success && !has_error;
}

bool World::is_valid() const
{
    return m_valid;
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
        auto result = s.sanity_checker().check(m_engine->workspace());

        if(result != SanityCheckResult::Success)
        {
            s.sanity_checker().report();
        }

        m_valid = (result == SanityCheckResult::Success);
    }
}
}  // namespace uipc::core
