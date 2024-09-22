#include <animator/animator.h>

namespace uipc::backend::cuda
{
void Animator::do_build()
{
    m_global_animator = &require<GlobalAnimator>();

    BuildInfo info;
    do_build(info);

    m_global_animator->register_animator(this);
}

void Animator::init()
{
    do_init();
}

void Animator::step()
{
    do_step();
}
}  // namespace uipc::backend::cuda
