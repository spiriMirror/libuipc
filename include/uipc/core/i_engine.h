#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/core/engine_status.h>
#include <uipc/core/feature_collection.h>

namespace uipc::core
{
class World;

class UIPC_CORE_API IEngine
{
  public:
    virtual ~IEngine() = default;
    void init(internal::World& w);
    void advance();
    void backward();
    void sync();
    void retrieve();
    Json to_json() const;

    bool                     dump();
    bool                     recover(SizeT dst_frame);
    SizeT                    frame() const;
    EngineStatusCollection&  status();
    const FeatureCollection& features() const;

  protected:
    virtual void                     do_init(internal::World&) = 0;
    virtual void                     do_advance()              = 0;
    virtual void                     do_backward()             = 0;
    virtual void                     do_sync()                 = 0;
    virtual void                     do_retrieve()             = 0;
    virtual Json                     do_to_json() const;
    virtual bool                     do_dump();
    virtual bool                     do_recover(SizeT dst_frame);
    virtual SizeT                    get_frame() const    = 0;
    virtual EngineStatusCollection&  get_status()         = 0;
    virtual const FeatureCollection& get_features() const = 0;
};
}  // namespace uipc::core
