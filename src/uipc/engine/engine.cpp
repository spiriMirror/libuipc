#include <uipc/engine/engine.h>
#include <dylib.hpp>

namespace uipc::engine
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

Json IEngine::do_to_json() const
{
    return Json{};
}
}  // namespace uipc::engine
