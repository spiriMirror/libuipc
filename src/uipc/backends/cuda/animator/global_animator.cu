#include <animator/global_animator.h>
#include <animator/animator.h>

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
