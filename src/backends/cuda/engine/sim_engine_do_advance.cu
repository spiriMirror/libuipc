#include <sim_engine.h>

namespace uipc::backend::cuda
{
void SimEngine::do_advance()
{
    // TODO: select pipeline
    advance_AL();
}
}  // namespace uipc::backend::cuda
