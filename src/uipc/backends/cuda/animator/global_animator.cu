#include <animator/global_animator.h>
#include <animator/animator.h>
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
        if(types.find(constitution::ConstitutionType::Constraint) == types.end())
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

void GlobalAnimator::do_build()
{
    const auto& scene = world().scene();
    auto&       types = scene.constitution_tabular().types();
    if(types.find(constitution::ConstitutionType::Constraint) == types.end())
    {
        throw SimSystemException("No Constraint found in the scene");
    }
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

    // update backend animator
    for(auto&& animator : m_animators.view())
    {
        animator->step();
    }
}

void GlobalAnimator::register_animator(Animator* animator)
{
    m_animators.register_subsystem(*animator);
}
}  // namespace uipc::backend::cuda
