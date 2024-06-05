#pragma once
#include <muda/ext/eigen/eigen_cxx20.h>
#include <uipc/backend/macro.h>
#include <uipc/engine/engine.h>
#include <sstream>
namespace uipc::backend
{
class UIPC_BACKEND_API CudaEngine : public engine::IEngine
{
    class DeviceCommon;

  public:
    CudaEngine();
    ~CudaEngine();

    // delete copy
    CudaEngine(const CudaEngine&)            = delete;
    CudaEngine& operator=(const CudaEngine&) = delete;

    DeviceCommon& device_common() noexcept;

  protected:
    void do_init(backend::WorldVisitor v) override;
    void do_advance() override;
    void do_sync() override;
    void do_retrieve() override;

    U<DeviceCommon> m_device_common;
    std::stringstream m_string_stream;
};
}  // namespace uipc::backend
