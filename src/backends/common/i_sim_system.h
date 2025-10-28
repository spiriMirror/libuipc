#pragma once
#include <string_view>
#include <uipc/common/type_define.h>
#include <uipc/common/json.h>
#include <uipc/common/exception.h>
#include <uipc/common/span.h>

namespace uipc::backend
{
class ISimSystem
{
  public:
    virtual ~ISimSystem() = default;
    std::string_view        name() const noexcept;
    bool                    is_engine_aware() const noexcept;
    bool                    is_valid() const noexcept;
    bool                    is_building() const noexcept;
    span<ISimSystem* const> strong_dependencies() const noexcept;
    span<ISimSystem* const> weak_dependencies() const noexcept;
    Json                    to_json() const;

    class BaseInfo
    {
      public:
        BaseInfo(SizeT frame, std::string_view workspace, const Json& config) noexcept;
        std::string_view workspace() const noexcept;
        std::string      dump_path(std::string_view file) const noexcept;
        const Json&      config() const noexcept;
        SizeT            frame() const noexcept;

      private:
        SizeT            m_frame = 0;
        std::string_view m_workspace;
        Json             m_config;
    };

    class DumpInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
    };

    class RecoverInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
    };

    /**
     * @brief Dump the simulation data to files
     * 
     * @return true if the dump is successful, false otherwise
     */
    bool dump(DumpInfo&);

    /**
     * @brief Try to recover the simulation data from files
     * 
     * SimSystem should store the information in temporary buffer, if the recovery fails, the buffer will be discarded
     * 
     * \return 
     */
    bool try_recover(RecoverInfo&);

    /**
     * @brief Apply the recovered data to the simulation
     * 
     * If some try_recover() fails, the apply_recover() will not be called
     */
    void apply_recover(RecoverInfo&);

    /**
	 * @brief Clear the valid recovered data
	 * 
	 * If some try_recover() fails, the clear_recover() will be called
	 */
    void clear_recover(RecoverInfo&);

    /**
     * @brief Overwrites vertex positions in the system.
     * 
     * E.g. used for resetting objects.
     */
    bool write_vertex_pos_to_sim(span<const Vector3> positions, IndexT vertex_offset, SizeT vertex_count); 

  protected:
    virtual void             do_build()       = 0;
    virtual std::string_view get_name() const = 0;
    virtual bool             do_dump(DumpInfo&);
    virtual bool             do_try_recover(RecoverInfo&);
    virtual void             do_apply_recover(RecoverInfo&);
    virtual void             do_clear_recover(RecoverInfo&);

    virtual bool do_write_vertex_pos_to_sim(span<const Vector3> positions, IndexT vertex_offset, SizeT vertex_count);
 

  private:
    friend class SimEngine;
    friend class SimSystemCollection;
    friend class SimSystem;

    void build();
    void make_engine_aware();
    void invalidate() noexcept;

    virtual void set_engine_aware() noexcept       = 0;
    virtual bool get_engine_aware() const noexcept = 0;
    virtual void set_invalid() noexcept            = 0;
    virtual bool get_valid() const noexcept        = 0;
    virtual Json do_to_json() const                = 0;
    virtual void set_building(bool b) noexcept     = 0;
    virtual bool get_is_building() const noexcept  = 0;
    virtual span<ISimSystem* const> get_strong_dependencies() const noexcept = 0;
    virtual span<ISimSystem* const> get_weak_dependencies() const noexcept = 0;
};

class SimSystemException : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::backend
