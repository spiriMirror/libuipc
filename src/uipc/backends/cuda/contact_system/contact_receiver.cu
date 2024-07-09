#include <contact_system/contact_receiver.h>

namespace uipc::backend::cuda
{
void ContactReceiver::do_build(BuildInfo& info) {}
void ContactReceiver::do_build()
{
    auto& global_contact_manager = require<GlobalContactManager>();

    BuildInfo info;
    do_build(info);

    global_contact_manager.add_receiver(this);
}
void ContactReceiver::report(GlobalContactManager::ClassifyInfo& info)
{
    do_report(info);
}
void ContactReceiver::receive(GlobalContactManager::ClassifiedContactInfo& info)
{
    do_receive(info);
}
}  // namespace uipc::backend::cuda
