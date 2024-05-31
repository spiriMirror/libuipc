#include <uipc/engine/engine.h>

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
}  // namespace uipc::engine
