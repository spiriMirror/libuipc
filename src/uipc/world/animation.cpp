#include <uipc/world/animation.h>
#include <uipc/world/world.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/geometry/implicit_geometry.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/simplicial_complex.h>
namespace uipc::world
{
Animation::Animation(World& world, Object& object, ActionOnUpdate&& on_update) noexcept
    : m_world(&world)
    , m_object(&object)
    , m_on_update(std::move(on_update))

{
}

void Animation::init()
{
    auto world   = backend::WorldVisitor{*m_world};
    auto scene   = world.scene();
    auto geo_ids = m_object->geometries().ids();
    for(auto id : geo_ids)
    {
        auto slot = scene.find_geometry(id);
        UIPC_ASSERT(slot, "Animation: Geometry slot not found for id={}", id);
        m_temp_geo_slots.push_back(slot);
        auto rest_slot = scene.find_rest_geometry(id);
        UIPC_ASSERT(rest_slot, "Animation: Rest geometry slot not found for id={}", id);
        m_temp_rest_geo_slots.push_back(rest_slot);
    }

    for(auto& slot : m_temp_geo_slots)
    {
        if(slot->geometry().type() == builtin::SimplicialComplex)
        {
            auto sc = slot->geometry().as<geometry::SimplicialComplex>();


            auto ins_is_fixed = sc->instances().find<IndexT>(builtin::is_fixed);


            auto vert_is_fixed = sc->vertices().find<IndexT>(builtin::is_fixed);


            auto both = ins_is_fixed && vert_is_fixed;

            UIPC_ASSERT(!both,
                        "Animation: SimplicialComplex {} has both fixed vertices and instances, which is ambiguous.",
                        slot->id());

            auto none = !ins_is_fixed && !vert_is_fixed;

            UIPC_ASSERT(!none,
                        "Animation: SimplicialComplex {} has neither fixed vertices nor instances, which is meaningless for animation.",
                        slot->id());

            if(ins_is_fixed)
            {
                // add aim_transform attribute, at this point, the value is shared with position
                sc->instances().share(builtin::aim_transform, sc->transforms());
            }
            else if(vert_is_fixed)
            {
                // add aim_position attribute, at this point, the value is shared with position
                sc->vertices().share(builtin::aim_position, sc->positions());
            }
        }
        else if(slot->geometry().type() == builtin::ImplicitGeometry)
        {
            auto ig = slot->geometry().as<geometry::ImplicitGeometry>();

            auto ins_is_fixed = ig->instances().find<IndexT>(builtin::is_fixed);

            UIPC_ASSERT(ins_is_fixed,
                        "Animation: ImplicitGeometry {} has no fixed instances, which is meaningless for animation.",
                        slot->id());

            // Do nothing for ImplicitGeometry, which is varying by its type.
        }
    }
}

void Animation::update()
{
    UpdateInfo info{*this};
    m_on_update(info);
}

Object& Animation::UpdateInfo::object() const noexcept
{
    return *(m_animation->m_object);
}

span<S<geometry::GeometrySlot>> Animation::UpdateInfo::geo_slots() const noexcept
{
    return m_animation->m_temp_geo_slots;
}

span<S<geometry::GeometrySlot>> Animation::UpdateInfo::rest_geo_slots() const noexcept
{
    return m_animation->m_temp_rest_geo_slots;
}

auto Animation::UpdateInfo::hint() noexcept -> UpdateHint&
{
    return m_hint;
}

Animation::UpdateInfo::UpdateInfo(Animation& animation) noexcept
    : m_animation(&animation)
{
}

void Animation::UpdateHint::fixed_vertices_changing(bool v) noexcept
{
    m_fixed_vertices_changing = v;
}
}  // namespace uipc::world
