#include <kernel_cout.h>
#include <muda/muda.h>
#include <muda/debug.h>
#include <uipc/common/log.h>

namespace uipc::backend::cuda
{
KernelCout::KernelCout()
{
    init();
}

void KernelCout::init()
{
    using namespace muda;

    Debug::set_sync_callback(
        [this]
        {
            m_string_stream.str("");
            m_logger.retrieve(m_string_stream);
            if(m_string_stream.str().empty())
                return;

            spdlog::info(R"( 
-------------------------------------------------------------------------------
*                               Kernel  Console                               *
-------------------------------------------------------------------------------
{}
-------------------------------------------------------------------------------)",
                         m_string_stream.str());
        });
}

muda::LoggerViewer KernelCout::viewer()
{
    // don't delete, just let it go with the program
    // or CUDA may crash at the end of the program
    thread_local static KernelCout* s_instance = nullptr;

    if(!s_instance)
    {
        s_instance = new KernelCout();
        s_instance->init();
    }

    return s_instance->_viewer();
}

muda::LoggerViewer KernelCout::_viewer()
{
    return m_logger.viewer();
}
}  // namespace uipc::backend::cuda