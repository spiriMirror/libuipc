#include <muda/ext/linear_system/type_mapper/algo_mapper.h>
namespace muda
{
template <typename T>
void LinearSystemContext::generic_spmv(const T&                  a,
                                       cusparseOperation_t       op,
                                       cusparseSpMatDescr_t      A,
                                       const cusparseDnVecDescr* x,
                                       const T&                  b,
                                       cusparseDnVecDescr_t      y)
{
    set_pointer_mode_host();

    size_t buffer_size = 0;
    checkCudaErrors(cusparseSpMV_bufferSize(
        cusparse(), op, &a, A, x, &b, y, cuda_data_type<T>(), LinearSystemAlgorithm::SPMV_ALG_DEFAULT, &buffer_size));

    auto buffer = temp_buffer(buffer_size);

    checkCudaErrors(cusparseSpMV(cusparse(),
                                 op,
                                 &a,
                                 A,
                                 x,
                                 &b,
                                 y,
                                 cuda_data_type<T>(),
                                 LinearSystemAlgorithm::SPMV_ALG_DEFAULT,
                                 buffer.data()));
}
}  // namespace muda

#include "spmv/coo_spmv.inl"
#include "spmv/csr_spmv.inl"
#include "spmv/bsr_spmv.inl"
#include "spmv/triplet_spmv.inl"