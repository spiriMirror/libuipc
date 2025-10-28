#pragma once
#include <uipc/backend/macro.h>
#include <uipc/core/i_engine.h>
#include <backends/common/sim_system_collection.h>
#include <backends/common/i_sim_system.h>
#include <uipc/core/engine_status.h>

namespace uipc::backend
{
class EngineCreateInfo;
class SimEngine : public core::IEngine
{
    friend class SimSystem;

  public:
    SimEngine(EngineCreateInfo* info);
    virtual ~SimEngine() = default;

    SimEngine(const SimEngine&)            = delete;
    SimEngine& operator=(const SimEngine&) = delete;

    WorldVisitor& world() noexcept;

  protected:
    virtual Json do_to_json() const override;

    /**
     * @brief Build the SimSystems in the engine.
     * 
     * This function will check the dependencies of each SimSystem and check the validity of the SimSystems.
     * If some SimSystems are invalid, any SimSystems that depend on them will also be invalidated.
     */
    void build_systems();

    /**
     * @brief Dump the system information to "systems.json" in engine working directory.
     */
    virtual void dump_system_info() const;

    /**
     * @brief Find certain SimSystem
     * 
     * Find certain SimSystem by type T.
     * 1. If the SimSystem is not found, return nullptr.
     * 2. If the SimSystem is invalid, return nullptr.
     * 3. Else, return the SimSystem.
     * 
     */
    template <std::derived_from<ISimSystem> T>
    T* find();

    /**
     * @brief Require certain SimSystem
     * 
     * Require certain SimSystem by type T.
     * 1. If the SimSystem is not found, throw SimEngineException.
     * 2. If the SimSystem is invalid, throw SimEngineException.
     * 3. Else, return the SimSystem.
     * 
     */
    template <std::derived_from<ISimSystem> T>
    T& require();

    /**
     * @brief Get the workspace of the engine, which is set by frontend user.
     * 
     * @return std::string_view 
     */
    std::string_view workspace() const noexcept;

    /**
     * @brief Return the path of the dump file.
     * 
     * @param _file_ Should always be __FILE__, __FILE__ should be a the backend c++ sources file.
     */
    std::string dump_path(std::string_view _file_) const noexcept;

    class InitInfo
    {
      public:
        Json& config() noexcept { return m_config; }

      private:
        friend class SimEngine;
        InitInfo(const Json& info)
            : m_config(info)
        {
        }
        Json m_config;
    };

    using DumpInfo    = ISimSystem::DumpInfo;
    using RecoverInfo = ISimSystem::RecoverInfo;

    virtual void do_init(InitInfo&)             = 0;
    virtual bool do_dump(DumpInfo&)             = 0;
    virtual bool do_try_recover(RecoverInfo&)   = 0;
    virtual void do_apply_recover(RecoverInfo&) = 0;
    virtual void do_clear_recover(RecoverInfo&) = 0;

    // virtual bool do_write_vertex_pos_to_sim(span<const Vector3> positions, span<const IndexT> geo_slot_indices) = 0;
    virtual bool do_write_vertex_pos_to_sim(span<const Vector3> positions, IndexT global_vertex_offset, IndexT local_vertex_offset, SizeT vertex_count, string system_name) final override;
   

    span<ISimSystem* const>  systems() noexcept;
    core::FeatureCollection& features() noexcept;

  private:
    virtual void do_init(core::internal::World& v) final override;
    virtual bool do_recover(SizeT dst_frame) final override;
    virtual bool do_dump() final override;
   
    ISimSystem*  find_system(ISimSystem* ptr);
    ISimSystem*  require_system(ISimSystem* ptr);
    virtual core::EngineStatusCollection&  get_status() final override;
    virtual const core::FeatureCollection& get_features() const final override;

    U<WorldVisitor>              m_world_visitor;
    SimSystemCollection          m_system_collection;
    std::string                  m_workspace;
    core::EngineStatusCollection m_status;
    core::FeatureCollection      m_features;
};

class SimEngineException : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::backend

#include "details/sim_engine.inl"
