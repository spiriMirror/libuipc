#pragma once
#include <pyuipc/pyuipc.h>
#include <uipc/core/engine.h>
#include <uipc/core/world.h>
#include <uipc/backend/visitors/world_visitor.h>

namespace pyuipc::core
{
class PyIEngine : public uipc::core::IEngine
{
  public:
    PyIEngine()                                                    = default;
    virtual void                                do_init()          = 0;
    virtual void                                do_advance()       = 0;
    virtual void                                do_sync()          = 0;
    virtual void                                do_retrieve()      = 0;
    virtual Json                                do_to_json() const = 0;
    virtual bool                                do_dump()          = 0;
    virtual bool                                do_recover(SizeT dst_frame) = 0;
    virtual SizeT                               get_frame() const           = 0;
    virtual uipc::core::EngineStatusCollection& get_status()                = 0;
    virtual const uipc::core::FeatureCollection& get_features() const       = 0;
    S<uipc::core::World> world() const { return m_world; }

  private:
    virtual void do_init(uipc::core::internal::World& w) final override;

    S<uipc::core::World> m_world;
};

// trampoline class
class PyIEngine_ : public PyIEngine
{
  public:
    using PyIEngine::PyIEngine;

    virtual void do_init() override
    {
        PYBIND11_OVERRIDE_PURE(void, PyIEngine_, do_init);
    };

    virtual void do_advance() override
    {
        PYBIND11_OVERRIDE_PURE(void, PyIEngine_, do_advance);
    };

    virtual void do_sync() override
    {
        PYBIND11_OVERRIDE_PURE(void, PyIEngine_, do_sync);
    };

    virtual void do_retrieve() override
    {
        PYBIND11_OVERRIDE_PURE(void, PyIEngine_, do_retrieve);
    };


    virtual Json do_to_json() const override
    {
        PYBIND11_OVERRIDE_PURE(Json, PyIEngine_, do_to_json);
    };

    virtual bool do_dump() override
    {
        PYBIND11_OVERRIDE_PURE(bool, PyIEngine_, do_dump);
    };

    virtual bool do_recover(SizeT dst_frame) override
    {
        PYBIND11_OVERRIDE_PURE(bool, PyIEngine_, do_recover, dst_frame);
    };

    virtual SizeT get_frame() const override
    {
        PYBIND11_OVERRIDE_PURE(SizeT, PyIEngine_, get_frame);
    };

    virtual uipc::core::EngineStatusCollection& get_status() override
    {
        return status;
    };

    virtual const uipc::core::FeatureCollection& get_features() const override
    {
        return features;
    };

    uipc::core::EngineStatusCollection status;
    uipc::core::FeatureCollection      features;
};
}  // namespace pyuipc::core
