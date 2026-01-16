#include <sim_engine.h>

namespace uipc::backend::cuda
{
void SimEngine::do_advance()
{
    switch(m_pipeline_type)
    {
        case SimEngine::PipelineType::Basic:
            advance();
            break;
        case SimEngine::PipelineType::AugmentedLagrangian:
            advance_AL();
            break;
        default:
            UIPC_ERROR_WITH_LOCATION("Unknown pipeline type");
            break;
    }
}
}  // namespace uipc::backend::cuda
