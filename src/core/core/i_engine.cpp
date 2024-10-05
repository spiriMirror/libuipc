#include <uipc/core/i_engine.h>
#include <dylib.hpp>

namespace uipc::core
{
void IEngine::init(backend::WorldVisitor v)
{
    do_init(v);
}

void IEngine::advance()
{
    do_advance();
}

void IEngine::sync()
{
    do_sync();
}

void IEngine::retrieve()
{
    do_retrieve();
}

Json IEngine::to_json() const
{
    return do_to_json();
}

bool IEngine::dump()
{
    return do_dump();
}

bool IEngine::recover()
{
    return do_recover();
}

SizeT IEngine::frame() const
{
    return get_frame();
}

Json IEngine::do_to_json() const
{
    return Json{};
}

bool IEngine::do_dump()
{
    return true;
}

bool IEngine::do_recover()
{
    return true;
}
}  // namespace uipc::core
