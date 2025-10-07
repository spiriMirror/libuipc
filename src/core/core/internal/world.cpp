#include <uipc/core/internal/world.h>
#include <uipc/core/internal/engine.h>
#include <uipc/core/sanity_checker.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/zip.h>
#include <uipc/common/uipc.h>
#include <dylib.hpp>
#include <uipc/backend/module_init_info.h>
#include <uipc/core/internal/scene.h>


namespace uipc::core::internal
{
static S<internal::Engine> lock(const W<internal::Engine>& e)
{
    UIPC_ASSERT(!e.expired(), "Engine is expired, did you throw the `Engine`?");
    return e.lock();
}

World::World(internal::Engine& e) noexcept
    : m_engine(e.weak_from_this())
{
}

void World::init(internal::Scene& s)
{
    if(m_scene)
        return;

    sanity_check(s);

    if(!m_valid)
    {
        log::error("World is not valid, skipping init.");
        return;
    }
    m_scene = s.shared_from_this();
    m_scene->init(*this);

    auto engine = lock(m_engine);

    engine->init(*this);

    if(engine->status().has_error())
    {
        log::error("Engine has error after init, world becomes invalid.");
        m_valid = false;
    }
}

void World::advance()
{
    if(!m_valid)
    {
        log::error("World is not valid, skipping advance.");
        return;
    }

    auto engine = lock(m_engine);

    engine->advance();

    if(engine->status().has_error())
    {
        log::error("Engine has error after advance, world becomes invalid.");
        m_valid = false;
    }
}

void World::sync()
{
    if(!m_valid)
    {
        log::error("World is not valid, skipping sync.");
        return;
    }

    auto engine = lock(m_engine);

    engine->sync();

    if(engine->status().has_error())
    {
        log::error("Engine has error after sync, world becomes invalid.");
        m_valid = false;
    }
}

void World::retrieve()
{
    if(!m_valid)
    {
        log::error("World is not valid, skipping retrieve.");
        return;
    }

    auto engine = lock(m_engine);

    engine->retrieve();

    if(engine->status().has_error())
    {
        log::error("Engine has error after retrieve, world becomes invalid.");
        m_valid = false;
    }
}

bool World::dump()
{
    if(!m_valid)
    {
        log::error("World is not valid, skipping dump.");
        return false;
    }

    auto engine = lock(m_engine);

    bool success   = engine->dump();
    bool has_error = engine->status().has_error();
    if(has_error)
    {
        log::error("Engine has error after dump, world becomes invalid.");
        m_valid = false;
    }

    return success && !has_error;
}

bool World::recover(SizeT aim_frame)
{
    if(!m_scene)
    {
        log::warn("Scene has not been set, skipping recover. Hint: you may call World::init() first.");
        return false;
    }

    if(!m_valid)
    {
        log::error("World is not valid, skipping recover.");
        return false;
    }

    auto engine = lock(m_engine);

    bool success   = engine->recover(aim_frame);
    bool has_error = engine->status().has_error();

    if(has_error)
    {
        log::error("Engine has error after recover, world becomes invalid.");
        m_valid = false;
    }

    if(success && !has_error)
    {
        // if diff_sim is not empty, broadcast parameters
        auto& diff_sim = m_scene->diff_sim();
        if(diff_sim.parameters().size() > 0)
            diff_sim.parameters().broadcast();
    }

    return success && !has_error;
}

bool World::is_valid() const
{
    return m_valid;
}

SizeT World::frame() const
{
    auto engine = lock(m_engine);

    if(!m_valid)
    {
        log::error("World is not valid, frame set to 0.");
        return 0;
    }
    return engine->frame();
}

const FeatureCollection& World::features() const
{
    auto engine = lock(m_engine);
    return engine->features();
}

void World::sanity_check(Scene& s)
{
    auto engine = lock(m_engine);

    auto& config                   = s.config();
    auto  sanity_check_enable_attr = config.find<IndexT>("sanity_check/enable");
    if(sanity_check_enable_attr->view()[0])
    {
        auto result = s.sanity_checker().check(engine->workspace());

        if(result != SanityCheckResult::Success)
        {
            s.sanity_checker().report();
        }

        m_valid = (result == SanityCheckResult::Success);
    }
}
}  // namespace uipc::core::internal
