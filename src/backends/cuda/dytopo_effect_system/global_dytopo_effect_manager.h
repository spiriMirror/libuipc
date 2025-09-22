#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <muda/ext/linear_system.h>
#include <utils/offset_count_collection.h>
#include <algorithm/matrix_converter.h>
#include <dytopo_effect_system/dytopo_classify_info.h>

namespace uipc::backend::cuda
{
class DyTopoEffectReporter;
class DyTopoEffectReceiver;

class GlobalDyTopoEffectManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class GradientHessianExtentInfo
    {
      public:
        void gradient_count(SizeT count) noexcept { m_gradient_count = count; }
        void hessian_count(SizeT count) noexcept { m_hessian_count = count; }

      private:
        friend class Impl;
        SizeT m_gradient_count;
        SizeT m_hessian_count;
    };

    class GradientHessianInfo
    {
      public:
        muda::DoubletVectorView<Float, 3> gradients() const noexcept
        {
            return m_gradients;
        }
        muda::TripletMatrixView<Float, 3> hessians() const noexcept
        {
            return m_hessians;
        }


      private:
        friend class Impl;
        muda::DoubletVectorView<Float, 3> m_gradients;
        muda::TripletMatrixView<Float, 3> m_hessians;
    };

    class EnergyExtentInfo
    {
      public:
        void energy_count(SizeT count) noexcept { m_energy_count = count; }

      private:
        friend class Impl;
        friend class DyTopoEffectLineSearchReporter;
        SizeT m_energy_count = 0;
    };

    class EnergyInfo
    {
      public:
        muda::BufferView<Float> energies() const { return m_energies; }
        bool                    is_initial() const { return m_is_initial; }

      private:
        friend class DyTopoEffectLineSearchReporter;
        muda::BufferView<Float> m_energies;
        bool                    m_is_initial = false;
    };

    using ClassifyInfo = DyTopoClassifyInfo;

    class ClassifiedDyTopoEffectInfo
    {
      public:
        muda::CDoubletVectorView<Float, 3> gradients() const noexcept
        {
            return m_gradients;
        }
        muda::CTripletMatrixView<Float, 3> hessians() const noexcept
        {
            return m_hessians;
        }

      private:
        friend class Impl;
        muda::CDoubletVectorView<Float, 3> m_gradients;
        muda::CTripletMatrixView<Float, 3> m_hessians;
    };

    class Impl
    {
      public:
        void init(WorldVisitor& world);
        void compute_dytopo_effect();
        void _assemble();
        void _convert_matrix();
        void _distribute();

        SimSystemSlot<GlobalVertexManager> global_vertex_manager;

        Float reserve_ratio = 1.1;


        /***********************************************************************
        *                              Reporter                                *
        ***********************************************************************/

        SimSystemSlotCollection<DyTopoEffectReporter> dytopo_effect_reporters;

        OffsetCountCollection<IndexT> reporter_energy_offsets_counts;
        OffsetCountCollection<IndexT> reporter_gradient_offsets_counts;
        OffsetCountCollection<IndexT> reporter_hessian_offsets_counts;

        muda::DeviceTripletMatrix<Float, 3> collected_dytopo_effect_hessian;
        muda::DeviceDoubletVector<Float, 3> collected_dytopo_effect_gradient;

        MatrixConverter<Float, 3>        matrix_converter;
        muda::DeviceBCOOMatrix<Float, 3> sorted_dytopo_effect_hessian;
        muda::DeviceBCOOVector<Float, 3> sorted_dytopo_effect_gradient;

        /***********************************************************************
        *                               Receiver                               *
        ***********************************************************************/

        SimSystemSlotCollection<DyTopoEffectReceiver> dytopo_effect_receivers;

        muda::DeviceVar<Vector2i>  gradient_range;
        muda::DeviceBuffer<IndexT> selected_hessian;
        muda::DeviceBuffer<IndexT> selected_hessian_offsets;

        vector<muda::DeviceTripletMatrix<Float, 3>> classified_dytopo_effect_hessians;
        vector<muda::DeviceDoubletVector<Float, 3>> classified_dytopo_effect_gradients;

        void loose_resize_entries(muda::DeviceTripletMatrix<Float, 3>& m, SizeT size);
        void loose_resize_entries(muda::DeviceDoubletVector<Float, 3>& v, SizeT size);
        template <typename T>
        void loose_resize(muda::DeviceBuffer<T>& buffer, SizeT size)
        {
            if(size > buffer.capacity())
            {
                buffer.reserve(size * reserve_ratio);
            }
            buffer.resize(size);
        }
    };

    muda::CBCOOVectorView<Float, 3> gradients() const noexcept;
    muda::CBCOOMatrixView<Float, 3> hessians() const noexcept;

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    friend class DyTopoEffectLineSearchReporter;

    void init();
    void compute_dytopo_effect();

    friend class DyTopoEffectReporter;
    void add_reporter(DyTopoEffectReporter* reporter);
    friend class DyTopoEffectReceiver;
    void add_receiver(DyTopoEffectReceiver* receiver);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda