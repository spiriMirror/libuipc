#include <uipc/core/constitution_tabular.h>
#include <uipc/builtin/constitution_uid_collection.h>
#include <algorithm>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/core/internal/scene.h>

namespace uipc::core
{
class ConstitutionTabular::Impl
{
  public:
    Impl() noexcept  = default;
    ~Impl() noexcept = default;

    void insert(const constitution::IConstitution& constitution)
    {
        UIPC_ASSERT(!m_is_sorted, "Cannot insert into a built ConstitutionTabular");
        m_uid_set.insert(constitution.uid());
        m_types.insert(std::string{constitution.type()});
    }

    span<U64> uids() const noexcept
    {
        sort_if_needed();
        return m_uids;
    }

    const set<std::string>& types() const noexcept { return m_types; }

    void insert(U64 uid)
    {
        const auto& uid_info = builtin::ConstitutionUIDCollection::instance().find(uid);
        m_uid_set.insert(uid);
        m_types.insert(std::string{uid_info.type});
    }

    void sort_if_needed() const noexcept
    {
        if(m_is_sorted)
            return;

        m_uids.resize(m_uid_set.size());
        std::ranges::copy(m_uid_set, m_uids.begin());
        std::sort(m_uids.begin(), m_uids.end());

        m_is_sorted = true;
    }

    void init(internal::Scene& scene)
    {
        auto geos      = scene.geometries().geometry_slots();
        auto rest_geos = scene.rest_geometries().geometry_slots();

        for(auto&& geo : geos)
        {
            // 1. Base Constitution UID
            {
                auto constitution_uid =
                    geo->geometry().meta().find<U64>(builtin::constitution_uid);

                if(constitution_uid)
                    insert(constitution_uid->view().front());
            }

            // 2. Extra Constitution UIDs
            {
                auto extra_constitution_uids =
                    geo->geometry().meta().find<VectorXu64>(builtin::extra_constitution_uids);

                if(extra_constitution_uids)
                {
                    const auto& uids = extra_constitution_uids->view().front();
                    for(auto uid : uids)
                        insert(uid);
                }
            }

            // 3. Constraint UIDs
            {
                auto constraint_uids =
                    geo->geometry().meta().find<VectorXu64>(builtin::constraint_uids);

                if(constraint_uids)
                {
                    const auto& uids = constraint_uids->view().front();
                    for(auto uid : uids)
                        insert(uid);
                }
            }
        }
    }

  private:
    mutable bool        m_is_sorted = false;
    mutable vector<U64> m_uids;
    set<U64>            m_uid_set;
    set<std::string>    m_types;
};


ConstitutionTabular::ConstitutionTabular() noexcept
    : m_impl(uipc::make_unique<Impl>())
{
}

ConstitutionTabular::~ConstitutionTabular() noexcept {}

void ConstitutionTabular::insert(const constitution::IConstitution& constitution)
{
    m_impl->insert(constitution);
}

span<U64> ConstitutionTabular::uids() const noexcept
{
    return m_impl->uids();
}

const set<std::string>& ConstitutionTabular::types() const noexcept
{
    return m_impl->types();
}

void ConstitutionTabular::init(internal::Scene& scene)
{
    m_impl->init(scene);
}
}  // namespace uipc::core
