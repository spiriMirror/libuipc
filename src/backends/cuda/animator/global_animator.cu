#include <animator/global_animator.h>
#include <animator/animator.h>
#include <uipc/builtin/constitution_type.h>
#include <sim_engine.h>

namespace uipc::backend
{
template <>
class backend::SimSystemCreator<cuda::GlobalAnimator>
{
  public:
    static U<cuda::GlobalAnimator> create(SimEngine& engine)
    {
        auto  scene = dynamic_cast<cuda::SimEngine&>(engine).world().scene();
        auto& types = scene.constitution_tabular().types();

        // Create GlobalAnimator if there are Constraints need animator support
        bool has_constraint =
            types.find(std::string{builtin::Constraint}) != types.end();

        if(!has_constraint)
        {
            return nullptr;
        }
        return uipc::make_unique<cuda::GlobalAnimator>(engine);
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalAnimator);

void GlobalAnimator::do_build() {}

Float GlobalAnimator::substep_ratio() noexcept
{
    return m_substep_ratio;
}

void GlobalAnimator::init()
{
    // init frontend animator
    world().animator().init();

    // init backend animator
    for(auto&& animator : m_animators.view())
    {
        animator->init();
    }
}

void GlobalAnimator::step()
{
    // update frontend animator
    world().animator().update();

    // after frontend update, reset substep ratio
    // prepare for the next newton iteration
    m_substep_ratio = 0.0;

    // update backend animator
    for(auto&& animator : m_animators.view())
    {
        animator->step();
    }
}

void GlobalAnimator::compute_substep_ratio(SizeT newton_iter)
{
    Float substep = static_cast<Float>(world().animator().substep());
    Float t       = (static_cast<Float>(newton_iter) + Float{1.0}) / substep;
    UIPC_ASSERT(substep > 0, "substep must be greater than 0");
    m_substep_ratio = std::min(t, Float{1.0});  // clamp t to [0, 1]
}

void GlobalAnimator::register_animator(Animator* animator)
{
    m_animators.register_sim_system(*animator);
}
}  // namespace uipc::backend::cuda
