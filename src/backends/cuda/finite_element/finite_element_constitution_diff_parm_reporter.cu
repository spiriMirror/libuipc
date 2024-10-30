#include <finite_element/finite_element_constitution_diff_parm_reporter.h>
#include <finite_element/finite_element_constitution.h>

#include <finite_element/codim_0d_constitution.h>
#include <finite_element/codim_1d_constitution.h>
#include <finite_element/codim_2d_constitution.h>
#include <finite_element/fem_3d_constitution.h>

namespace uipc::backend::cuda
{
U64 FiniteElementConstitutionDiffParmReporter::uid() const noexcept
{
    return get_uid();
}

IndexT FiniteElementConstitutionDiffParmReporter::dim() const noexcept
{
    return get_dim();
}

void FiniteElementConstitutionDiffParmReporter::do_build(FiniteElementDiffParmReporter::BuildInfo& info)
{
    auto& fem = require<FiniteElementMethod>();

    BuildInfo this_info;
    do_build(this_info);

    fem.add_reporter(this);
}

FiniteElementConstitution& FiniteElementConstitutionDiffParmReporter::constitution() noexcept
{
    return *m_constitution;
}

const FiniteElementMethod::ConstitutionInfo& FiniteElementConstitutionDiffParmReporter::constitution_info() noexcept
{
    return m_constitution->constitution_info();
}

void FiniteElementConstitutionDiffParmReporter::connect()
{
    auto this_uid = uid();
    auto this_dim = dim();

    auto& impl = fem();

    auto connect_constitution = [this, this_uid, this_dim](auto& constitutions, auto& uid_to_index)
    {
        auto iter = uid_to_index.find(this_uid);
        UIPC_ASSERT(iter != uid_to_index.end(),
                    "Codim {}D UID ({}) not found, DiffParmReporter name = {}",
                    this_dim,
                    this_uid,
                    name());
        auto index     = iter->second;
        m_constitution = constitutions[index];
    };

    switch(this_dim)
    {
        case 0:
            connect_constitution(impl.codim_0d_constitutions, impl.codim_0d_uid_to_index);
            break;
        case 1:
            connect_constitution(impl.codim_1d_constitutions, impl.codim_1d_uid_to_index);
            break;
        case 2:
            connect_constitution(impl.codim_2d_constitutions, impl.codim_2d_uid_to_index);
            break;
        case 3:
            connect_constitution(impl.fem_3d_constitutions, impl.fem_3d_uid_to_index);
            break;
        default:
            UIPC_ASSERT(false, "Invalid Dim {}D", this_dim);
            break;
    }

    UIPC_ASSERT(m_constitution != nullptr,
                "DiffParmReporter {} Failed to connect to the constitution",
                name());
}

void FiniteElementConstitutionDiffParmReporter::init()
{
    FiniteElementMethod::FilteredInfo info{&fem(), dim(), m_constitution->m_index_in_dim};
    do_init(info);
}
}  // namespace uipc::backend::cuda
