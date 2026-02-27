namespace muda
{
template <typename T>
void muda::LinearSystemContext::sysv(DenseMatrixView<T> A, DenseVectorView<T> b)
{
    auto cusolver = cusolver_dn();

    auto info = std::make_shared<DeviceVar<int>>();


    size_t d_lwork = 0; /* size of workspace in device */
    size_t h_lwork = 0; /* size of workspace in host */

    cublasFillMode_t uplo = CUBLAS_FILL_MODE_LOWER;

    auto m = A.row();

    // query working space
    checkCudaErrors(cusolverDnXpotrf_bufferSize(
        cusolver, nullptr, uplo, m, cuda_data_type<T>(), A.data(), A.lda(), cuda_data_type<T>(), &d_lwork, &h_lwork));

    auto device_buffer = temp_buffer<T>(d_lwork);
    auto host_buffer   = temp_host_buffer<T>(h_lwork);


    // Cholesky factorization
    checkCudaErrors(cusolverDnXpotrf(cusolver,
                                     nullptr,
                                     uplo,
                                     m,
                                     cuda_data_type<T>(),
                                     A.data(),
                                     A.lda(),
                                     cuda_data_type<T>(),
                                     device_buffer.data(),
                                     d_lwork,
                                     host_buffer.data(),
                                     h_lwork,
                                     info->data()));

    // solve the system
    checkCudaErrors(cusolverDnXpotrs(cusolver,
                                     nullptr,
                                     uplo,
                                     m,
                                     1, /* nrhs */
                                     cuda_data_type<T>(),
                                     A.data(),
                                     A.lda(),
                                     cuda_data_type<T>(),
                                     b.data(),
                                     m,
                                     info->data()));

    std::string label{this->label()};
    this->label("");  // remove label because we consume it here

    add_sync_callback(
        [info = std::move(info), label = std::move(label)]() mutable
        {
            int result = *info;
            if(result < 0)
                MUDA_KERNEL_WARN_WITH_LOCATION("In calling label %s: A*x=b solving failed. The %d-th parameter in Cholesky factorization is wrong.",
                                               label.c_str(),
                                               -result);
        });
}

template <typename T>
void LinearSystemContext::gesv(DenseMatrixView<T> A, DenseVectorView<T> b)
{
    auto cusolver = cusolver_dn();

    int64_t m       = A.row();
    size_t  d_lwork = 0; /* size of workspace in device */
    size_t  h_lwork = 0; /* size of workspace in host */

    auto info = std::make_shared<DeviceVar<int>>();

    cusolverDnParams_t params;
    cusolverDnCreateParams(&params);
    cusolverDnSetAdvOptions(params, CUSOLVERDN_GETRF, CUSOLVER_ALG_0);

    // constexpr int pivot_on    = 1;
    size_t d_piv_count = A.row();

    checkCudaErrors(cusolverDnXgetrf_bufferSize(cusolver,
                                                params,
                                                A.row(),
                                                A.col(),
                                                cuda_data_type<T>(),
                                                A.data(),
                                                A.lda(),
                                                cuda_data_type<T>(),
                                                &d_lwork,
                                                &h_lwork));

    auto buffer = temp_buffer(d_lwork * sizeof(T) + d_piv_count * sizeof(int64_t));

    auto device_buffer = muda::BufferView<T>{(T*)buffer.data(), 0, d_lwork};

    auto last = device_buffer.data(d_lwork);

    auto d_piv = muda::BufferView<int64_t>{(int64_t*)(last), 0, d_piv_count};

    auto host_buffer = temp_host_buffer<T>(h_lwork);

    checkCudaErrors(cusolverDnXgetrf(cusolver,
                                     params,
                                     m,
                                     m,
                                     cuda_data_type<T>(),
                                     A.data(),
                                     A.lda(),
                                     d_piv.data(),
                                     cuda_data_type<T>(),
                                     device_buffer.data(),
                                     d_lwork,
                                     host_buffer.data(),
                                     h_lwork,
                                     info->data()));

    checkCudaErrors(cusolverDnXgetrs(cusolver,
                                     params,
                                     CUBLAS_OP_N,
                                     m,
                                     1, /* nrhs */
                                     cuda_data_type<T>(),
                                     A.data(),
                                     A.lda(),
                                     d_piv.data(),
                                     cuda_data_type<T>(),
                                     b.data(),
                                     m,
                                     info->data()));

    std::string label{this->label()};
    this->label("");  // remove label because we consume it here

    add_sync_callback(
        [info = std::move(info), label = std::move(label), params]() mutable
        {
            int result = *info;
            if(result < 0)
                MUDA_KERNEL_WARN_WITH_LOCATION("In calling label %f: A*x=b solving failed. The %d-th parameter in LU factorization is wrong.",
                                               label.c_str(),
                                               -result);

            checkCudaErrors(cusolverDnDestroyParams(params));
        });
}


template <typename T>
void LinearSystemContext::solve(DenseMatrixView<T> A_to_fact, DenseVectorView<T> b_to_x)
{
    if(A_to_fact.is_sym())
    {
        sysv(A_to_fact, b_to_x);
    }
    else
    {
        gesv(A_to_fact, b_to_x);
    }
}
}  // namespace muda