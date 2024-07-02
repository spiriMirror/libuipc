#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <muda/ext/linear_system.h>

namespace uipc::backend::cuda
{
class ContactReporter;
class ContactReceiver;

class GlobalContactManager : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class ContactExtentInfo
    {
      public:
        void gradient_count(SizeT count) { m_gradient_count = count; }
        void hessian_count(SizeT count) { m_hessian_count = count; }

      private:
        friend class Impl;
        SizeT m_gradient_count;
        SizeT m_hessian_count;
    };

    class ContactInfo
    {
      public:
        muda::DoubletVectorView<Float, 3> gradient() { return m_gradient; }
        muda::TripletMatrixView<Float, 3> hessian() { return m_hessian; }

      private:
        friend class Impl;
        muda::DoubletVectorView<Float, 3> m_gradient;
        muda::TripletMatrixView<Float, 3> m_hessian;
    };


    class Impl
    {
      public:
        void init();
        void compute_d_hat();
        void compute_adaptive_kappa();

        void assemble();

        GlobalVertexManager* global_vertex_manager = nullptr;

        list<ContactReporter*>   contact_reporter_buffer;
        vector<ContactReporter*> contact_reporters;
        list<ContactReceiver*>   contact_receiver_buffer;
        vector<ContactReceiver*> contact_receivers;

        vector<SizeT> reporter_hessian_counts;
        vector<SizeT> reporter_hessian_offsets;
        vector<SizeT> reporter_gradient_counts;
        vector<SizeT> reporter_gradient_offsets;


        Float d_hat         = 0.0;
        Float related_d_hat = 0.0;
        Float kappa         = 0.0;

        muda::DeviceTripletMatrix<Float, 3> collected_contact_hessian;
        muda::DeviceBCOOMatrix<Float, 3>    sorted_contact_hessian;
        muda::DeviceTripletMatrix<Float, 3> classified_contact_hessian;

        muda::DeviceDoubletVector<Float, 3> collected_contact_gradient;
        muda::DeviceBCOOVector<Float, 3>    sorted_contact_gradient;
        muda::DeviceDoubletVector<Float, 3> classified_contact_gradient;
    };

    Float d_hat() const;

    void add_reporter(ContactReporter* reporter);
    void add_receiver(ContactReceiver* receiver);

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    friend class ContactLineSearchReporter;
    void compute_d_hat();
    void compute_contact();
    void compute_adaptive_kappa();

    Impl m_impl;
};
}  // namespace uipc::backend::cuda