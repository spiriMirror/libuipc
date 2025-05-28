#pragma once
#include <sim_system.h>
#include <muda/buffer/buffer_view.h>
#include <utils/offset_count_collection.h>

namespace uipc::backend::cuda
{
class BodyReporter;
class GlobalBodyManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class BodyCountInfo
    {
      public:
        void count(SizeT count) noexcept;
        void changeable(bool is_changable) noexcept;

      private:
        friend class GlobalBodyManager;
        friend class BodyReporter;
        SizeT m_count     = 0;
        bool  m_changable = false;
    };

    class BodyAttributeInfo
    {
      public:
        muda::BufferView<IndexT> coindices() const noexcept;
        muda::BufferView<IndexT> self_collision() const noexcept;

      private:
        BodyAttributeInfo(Impl* impl, SizeT index) noexcept;
        friend class GlobalBodyManager;
        SizeT m_index;
        Impl* m_impl;
    };

    /**
     * @brief A mapping from the global body index to the coindices.
     * 
     * The values of coindices is dependent on the reporters, which can be:
     * 1) the local index of the body
     * 2) or any other information that is needed to be stored.
     */
    muda::CBufferView<IndexT> coindices() const noexcept;


    /*
     * @breif Tells if the body needs self-collision detection.
     * 
     * Indexed by body index, 1 for self-collision, 0 for no self-collision.
     */
    muda::CBufferView<IndexT> self_collision() const noexcept;

  public:
    class Impl
    {
      public:
        void init();
        void rebuild();

        template <typename T>
        muda::BufferView<T> subview(muda::DeviceBuffer<T>& buffer, SizeT index) const noexcept;

        muda::DeviceBuffer<IndexT> coindices;
        muda::DeviceBuffer<IndexT> self_collision;

        OffsetCountCollection<IndexT> reporter_body_offsets_counts;

        SimSystemSlotCollection<BodyReporter> body_reporters;
    };

  private:
    virtual void do_build() final override;

    friend class SimEngine;
    void init();  // only be called by SimEngine

    friend class BodyReporter;
    void add_reporter(BodyReporter* reporter);  // only be called by BodyReporter

    Impl m_impl;
};
}  // namespace uipc::backend::cuda