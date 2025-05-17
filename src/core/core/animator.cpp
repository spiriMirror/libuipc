#include <uipc/core/animator.h>
#include <uipc/common/log.h>
namespace uipc::core
{
void Animator::substep(SizeT n) noexcept
{
    UIPC_ASSERT(n > 0, "Animator: Substep must be greater than 0, yours' {}", n);
    m_substep = n;
}

SizeT Animator::substep() const noexcept
{
    return m_substep;
}

void Animator::insert(Object& obj, Animation::ActionOnUpdate&& on_update)
{
    if constexpr(uipc::RUNTIME_CHECK)
    {
        auto it = m_animations.find(obj.id());

        if(it != m_animations.end())
        {
            auto obj = it->second.m_object;
            UIPC_ASSERT(it == m_animations.end(),
                        "Animator: Object (name={}, id={}) already has an animation.",
                        obj->name(),
                        obj->id());
        }
    }

    m_animations.emplace(obj.id(), Animation(m_scene, obj, std::move(on_update)));
}

void Animator::erase(IndexT id)
{
    if constexpr(uipc::RUNTIME_CHECK)
    {
        auto it = m_animations.find(id);
        if(it == m_animations.end())
        {
            UIPC_WARN_WITH_LOCATION("Animator: No animation found for object id={}, ignore.", id);
        }
    }
    m_animations.erase(id);
}

Animator::Animator(internal::Scene& scene) noexcept
    : m_scene(scene)
{
}
}  // namespace uipc::core

namespace fmt
{
appender fmt::formatter<uipc::core::Animator>::format(const uipc::core::Animator& c,
                                                      format_context& ctx) const
{

    fmt::format_to(ctx.out(), "Animations({}):", c.m_animations.size());

    for(auto& [id, animation] : c.m_animations)
    {
        fmt::format_to(ctx.out(), "\n  {}", animation);
    }

    return ctx.out();
}
}  // namespace fmt