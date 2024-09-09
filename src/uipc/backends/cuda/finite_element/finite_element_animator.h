#pragma once
#include <animator/animator.h>
#include <finite_element/finite_element_method.h>
#include <gradient_hessian_computer.h>
#include <line_search/line_searcher.h>

namespace uipc::backend::cuda
{
class FiniteElementConstraint;
class FiniteElementAnimator : public Animator
{
  public:
    using Animator::Animator;

    using AnimatedGeoInfo = FiniteElementMethod::GeoInfo;

    class Impl;

    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl, SizeT index)
            : m_impl(impl)
            , m_index(index)
        {
        }

        span<const AnimatedGeoInfo> animated_geo_infos() const;

        span<const IndexT> anim_indices() const;

        template <typename ForEach, typename ViewGetter>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ViewGetter&&                    view_getter,
                      ForEach&&                       for_each_action) noexcept;

      private:
        Impl* m_impl  = nullptr;
        SizeT m_index = ~0ull;
    };

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl, SizeT index, Float dt)
            : m_impl(impl)
            , m_index(index)
            , m_dt(dt)
        {
        }

        Float dt() const noexcept;

        muda::CBufferView<IndexT> anim_indices() const noexcept;

        muda::CBufferView<Vector3> xs() const noexcept;
        muda::CBufferView<Float>   masses() const noexcept;
        muda::CBufferView<FiniteElementMethod::FixType> is_fixed() const noexcept;

      protected:
        Impl* m_impl  = nullptr;
        SizeT m_index = ~0ull;
        Float m_dt;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
        muda::BufferView<Float> energies() const noexcept;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        using BaseInfo::BaseInfo;
        muda::BufferView<Vector3>   gradients() const noexcept;
        muda::BufferView<Matrix3x3> hessians() const noexcept;
    };

    class Impl
    {
      public:
        void init(backend::WorldVisitor& world);
        void step();

        Float dt;

        FiniteElementMethod* finite_element_method = nullptr;
        SimSystemSlotCollection<FiniteElementConstraint> constraints;
        unordered_map<U64, SizeT> uid_to_constraint_index;

        vector<AnimatedGeoInfo> anim_geo_infos;
        vector<SizeT>           constraint_geo_info_offsets;
        vector<SizeT>           constraint_geo_info_counts;

        vector<SizeT> constraint_vertex_counts;
        vector<SizeT> constraint_vertex_offsets;

        vector<IndexT> h_anim_indices;
        muda::DeviceBuffer<IndexT> anim_indices;  // view(anim_vertex_offsets[index], anim_vertex_counts[index])
    };

  private:
    friend class FiniteElementConstraint;
    void add_constraint(FiniteElementConstraint* constraint);

    friend class FEMLineSearchReporter;
    void compute_energy(LineSearcher::EnergyInfo& info);

    friend class FEMGradientHessianComputer;
    void compute_gradient_hessian(GradientHessianComputer::ComputeInfo& info);

    Impl m_impl;

    virtual void do_init() override;
    virtual void do_step() override;
    virtual void do_build(BuildInfo& info) override final;
};
}  // namespace uipc::backend::cuda


#include "details/finite_element_animator.inl"