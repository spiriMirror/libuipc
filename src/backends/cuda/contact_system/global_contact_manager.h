#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <muda/ext/linear_system.h>
#include <contact_system/contact_coeff.h>
#include <algorithm/matrix_converter.h>

namespace uipc::backend::cuda
{
class ContactReporter;
class ContactReceiver;
class GlobalTrajectoryFilter;

class GlobalContactManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class ContactExtentInfo
    {
      public:
        void gradient_count(SizeT count) noexcept { m_gradient_count = count; }
        void hessian_count(SizeT count) noexcept { m_hessian_count = count; }

      private:
        friend class Impl;
        SizeT m_gradient_count;
        SizeT m_hessian_count;
    };

    class ContactInfo
    {
      public:
        muda::DoubletVectorView<Float, 3> gradient() const noexcept
        {
            return m_gradient;
        }
        muda::TripletMatrixView<Float, 3> hessian() const noexcept
        {
            return m_hessian;
        }

      private:
        friend class Impl;
        muda::DoubletVectorView<Float, 3> m_gradient;
        muda::TripletMatrixView<Float, 3> m_hessian;
    };

    class EnergyInfo
    {
      public:
        muda::VarView<Float> energy() const { return m_energy; }
        bool                 is_initial() const { return m_is_initial; }

      private:
        friend class ContactLineSearchReporter;
        muda::VarView<Float> m_energy;
        bool                 m_is_initial = false;
    };

    class ClassifyInfo
    {
        enum class Type
        {
            Range,
            // MultiRange
        };

      public:
        /**
         * @brief The range of contact hessian i,j, and gradient range is empty. (Off-diagnonal)
         * 
         * $$ i \in [LBegin, LEnd) $$, $$ j \in [RBegin, REnd) $$. 
         * Contact hessian $H_{ij}$ will be passed to the reporter later.
         * 
         * @param LRange LRange=[LBegin, LEnd)
         * @param RRange RRange=[RBegin, REnd)
         */
        void range(const Vector2i& LRange, const Vector2i& RRange);

        /**
         * @brief The range of contact hessian and gradient. (Diagnonal)
         * 
         * $$ i \in [Begin, End) $$.
         * Contact gradient $G_{i}$ will be passed to the reporter later.
         * 
         * @param Range Range=[Begin, End)
         */
        void range(const Vector2i& Range);

      private:
        friend class Impl;
        friend class ContactReceiver;
        Vector2i m_hessian_i_range  = {0, 0};
        Vector2i m_hessian_j_range  = {0, 0};
        Vector2i m_gradient_i_range = {0, 0};
        Type     m_type;

        bool is_empty() const;

        bool is_diag() const;

        void sanity_check();
    };

    class ClassifiedContactInfo
    {
      public:
        muda::CDoubletVectorView<Float, 3> gradient() const noexcept
        {
            return m_gradient;
        }
        muda::CTripletMatrixView<Float, 3> hessian() const noexcept
        {
            return m_hessian;
        }

      private:
        friend class Impl;
        friend class ContactReceiver;
        muda::CDoubletVectorView<Float, 3> m_gradient;
        muda::CTripletMatrixView<Float, 3> m_hessian;
    };

    class Impl
    {
      public:
        void  init(WorldVisitor& world);
        void  compute_d_hat();
        void  compute_adaptive_kappa();
        Float compute_cfl_condition();

        void compute_contact();
        void _assemble();
        void _convert_matrix();
        void _distribute();

        SimSystemSlot<GlobalVertexManager>    global_vertex_manager;
        SimSystemSlot<GlobalTrajectoryFilter> global_trajectory_filter;

        bool cfl_enabled = false;

        vector<ContactCoeff>               h_contact_tabular;
        muda::DeviceBuffer2D<ContactCoeff> contact_tabular;
        vector<IndexT>                     h_contact_mask_tabular;
        muda::DeviceBuffer2D<IndexT>       contact_mask_tabular;
        Float                              reserve_ratio = 1.1;

        Float d_hat        = 0.0;
        Float kappa        = 0.0;
        Float dt           = 0.0;
        Float eps_velocity = 0.0;

        /***********************************************************************
        *                     Global Vertex Contact Info                       *
        ***********************************************************************/

        muda::DeviceBuffer<IndexT> vert_is_active_contact;
        muda::DeviceBuffer<Float>  vert_disp_norms;
        muda::DeviceVar<Float>     max_disp_norm;


        /***********************************************************************
        *                         Contact Reporter                             *
        ***********************************************************************/

        SimSystemSlotCollection<ContactReporter> contact_reporters;

        vector<SizeT> reporter_hessian_counts;
        vector<SizeT> reporter_hessian_offsets;
        vector<SizeT> reporter_gradient_counts;
        vector<SizeT> reporter_gradient_offsets;

        muda::DeviceTripletMatrix<Float, 3> collected_contact_hessian;
        muda::DeviceDoubletVector<Float, 3> collected_contact_gradient;

        MatrixConverter<Float, 3>        matrix_converter;
        muda::DeviceBCOOMatrix<Float, 3> sorted_contact_hessian;
        muda::DeviceBCOOVector<Float, 3> sorted_contact_gradient;

        /***********************************************************************
        *                         Contact Reporter                             *
        ***********************************************************************/

        SimSystemSlotCollection<ContactReceiver> contact_receivers;

        muda::DeviceVar<Vector2i>  gradient_range;
        muda::DeviceBuffer<IndexT> selected_hessian;
        muda::DeviceBuffer<IndexT> selected_hessian_offsets;

        vector<muda::DeviceTripletMatrix<Float, 3>> classified_contact_hessians;
        vector<muda::DeviceDoubletVector<Float, 3>> classified_contact_gradients;

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

    Float d_hat() const;
    Float eps_velocity() const;
    bool  cfl_enabled() const;

    muda::CBuffer2DView<ContactCoeff> contact_tabular() const noexcept;
    muda::CBuffer2DView<IndexT>       contact_mask_tabular() const noexcept;

    muda::CBCOOVectorView<Float, 3> contact_gradient() const noexcept;
    muda::CBCOOMatrixView<Float, 3> contact_hessian() const noexcept;

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    friend class ContactLineSearchReporter;
    friend class GlobalTrajectoryFilter;
    friend class ContactSystemExporter;

    void init();

    void  compute_d_hat();
    void  compute_contact();
    void  compute_adaptive_kappa();
    Float compute_cfl_condition();

    friend class ContactReporter;
    void add_reporter(ContactReporter* reporter);
    friend class ContactReceiver;
    void add_receiver(ContactReceiver* receiver);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda