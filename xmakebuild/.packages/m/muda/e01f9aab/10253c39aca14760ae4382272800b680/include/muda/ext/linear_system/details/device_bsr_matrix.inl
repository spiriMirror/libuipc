#include <muda/check/check_cusparse.h>
#include <muda/ext/linear_system/type_mapper/data_type_mapper.h>

namespace muda
{
template <typename Ty, int N>
DeviceBSRMatrix<Ty, N>::~DeviceBSRMatrix()
{
    destroy_all_descr();
}

template <typename Ty, int N>
DeviceBSRMatrix<Ty, N>::DeviceBSRMatrix(const DeviceBSRMatrix& other)
    : m_row(other.m_row)
    , m_col(other.m_col)
    , m_row_offsets(other.m_row_offsets)
    , m_col_indices(other.m_col_indices)
    , m_values(other.m_values)
{
}

template <typename Ty, int N>
DeviceBSRMatrix<Ty, N>::DeviceBSRMatrix(DeviceBSRMatrix&& other)
    : m_row(other.m_row)
    , m_col(other.m_col)
    , m_row_offsets(std::move(other.m_row_offsets))
    , m_col_indices(std::move(other.m_col_indices))
    , m_values(std::move(other.m_values))
    , m_legacy_descr(other.m_legacy_descr)
{
    other.m_row          = 0;
    other.m_col          = 0;
    other.m_legacy_descr = nullptr;
    other.m_descr        = nullptr;
}

template <typename Ty, int N>
DeviceBSRMatrix<Ty, N>& DeviceBSRMatrix<Ty, N>::operator=(const DeviceBSRMatrix& other)
{
    if(this != &other)
    {
        m_row         = other.m_row;
        m_col         = other.m_col;
        m_row_offsets = other.m_row_offsets;
        m_col_indices = other.m_col_indices;
        m_values      = other.m_values;

        destroy_all_descr();

        m_legacy_descr = nullptr;
        m_descr        = nullptr;
    }
    return *this;
}

template <typename Ty, int N>
DeviceBSRMatrix<Ty, N>& DeviceBSRMatrix<Ty, N>::operator=(DeviceBSRMatrix&& other)
{
    if(this != &other)
    {
        m_row         = other.m_row;
        m_col         = other.m_col;
        m_row_offsets = std::move(other.m_row_offsets);
        m_col_indices = std::move(other.m_col_indices);
        m_values      = std::move(other.m_values);

        destroy_all_descr();

        m_legacy_descr = other.m_legacy_descr;
        m_descr        = other.m_descr;

        other.m_row          = 0;
        other.m_col          = 0;
        other.m_legacy_descr = nullptr;
        other.m_descr        = nullptr;
    }
    return *this;
}

template <typename Ty, int N>
void DeviceBSRMatrix<Ty, N>::reshape(int row, int col)
{
    m_row = row;
    m_row_offsets.resize(row + 1);
    m_col   = col;
    m_descr = nullptr;
}

template <typename Ty, int N>
void DeviceBSRMatrix<Ty, N>::reserve(int non_zero_blocks)
{
    m_col_indices.reserve(non_zero_blocks);
    m_values.reserve(non_zero_blocks);
}

template <typename Ty, int N>
void DeviceBSRMatrix<Ty, N>::reserve_offsets(int size)
{
    m_row_offsets.reserve(size);
}

template <typename Ty, int N>
void DeviceBSRMatrix<Ty, N>::resize(int non_zero_blocks)
{
    m_col_indices.resize(non_zero_blocks);
    m_values.resize(non_zero_blocks);
}

template <typename Ty, int N>
void DeviceBSRMatrix<Ty, N>::clear()
{
    m_row         = 0;
    m_col         = 0;
    m_row_offsets = decltype(m_row_offsets)();
    m_col_indices = decltype(m_col_indices)();
    m_values      = decltype(m_values)();
    destroy_all_descr();
}

template <typename Ty, int N>
void DeviceBSRMatrix<Ty, N>::destroy_all_descr() const
{
    if(m_legacy_descr)
    {
        checkCudaErrors(cusparseDestroyMatDescr(m_legacy_descr));
        m_legacy_descr = nullptr;
    }
    if(m_descr)
    {
        checkCudaErrors(cusparseDestroySpMat(m_descr));
        m_descr = nullptr;
    }
}

template <typename Ty, int N>
cusparseSpMatDescr_t DeviceBSRMatrix<Ty, N>::descr() const
{
    if(m_descr == nullptr)
    {
        if constexpr(IsBlockMatrix)
        {
            //checkCudaErrors(cusparseCreateMatDescr(&m_legacy_descr));
            checkCudaErrors(cusparseCreateBsr(
                &m_descr,
                m_row,
                m_col,
                m_values.size(),
                N,
                N,
                remove_const(m_row_offsets.data()),
                remove_const(m_col_indices.data()),
                remove_const(m_values.data()),
                cusparse_index_type<typename decltype(m_row_offsets)::value_type>(),
                cusparse_index_type<typename decltype(m_col_indices)::value_type>(),
                CUSPARSE_INDEX_BASE_ZERO,
                cuda_data_type<Ty>(),
                cusparseOrder_t::CUSPARSE_ORDER_COL));
        }
        else
        {
            checkCudaErrors(cusparseCreateCsr(
                &m_descr,
                this->m_row,
                this->m_col,
                this->m_values.size(),
                remove_const(this->m_row_offsets.data()),
                remove_const(this->m_col_indices.data()),
                remove_const(this->m_values.data()),
                cusparse_index_type<typename decltype(this->m_row_offsets)::value_type>(),
                cusparse_index_type<typename decltype(this->m_col_indices)::value_type>(),
                CUSPARSE_INDEX_BASE_ZERO,
                cuda_data_type<Ty>()));
        }
    }
    return m_descr;
}

template <typename Ty, int N>
cusparseMatDescr_t DeviceBSRMatrix<Ty, N>::legacy_descr() const
{
    if(m_legacy_descr == nullptr)
    {
        if constexpr(IsBlockMatrix)
        {
            checkCudaErrors(cusparseCreateMatDescr(&m_legacy_descr));
        }
        else
        {
            checkCudaErrors(cusparseCreateMatDescr(&m_legacy_descr));
            checkCudaErrors(cusparseSetMatType(m_legacy_descr, CUSPARSE_MATRIX_TYPE_GENERAL));
            checkCudaErrors(cusparseSetMatIndexBase(m_legacy_descr, CUSPARSE_INDEX_BASE_ZERO));
            checkCudaErrors(cusparseSetMatDiagType(m_legacy_descr, CUSPARSE_DIAG_TYPE_NON_UNIT));
        }
    }

    return m_legacy_descr;
}
}  // namespace muda
