#include <uipc/core/diff_sim.h>

namespace uipc::core
{
diff_sim::ParameterCollection& DiffSim::parameters()
{
    return m_parameters;
}
const diff_sim::ParameterCollection& DiffSim::parameters() const
{
    return m_parameters;
}
}  // namespace uipc::core
