#include <contact_system/constitutions/ipc_simplex_contact.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(IPCSimplexContact);

void IPCSimplexContact::do_build(BuildInfo& info) 
{

}

void IPCSimplexContact::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
}

void IPCSimplexContact::do_assemble(ContactInfo& info) 
{
}
}  // namespace uipc::backend::cuda