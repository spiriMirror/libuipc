#include <torch/torch.h>

namespace uipc::torch
{
using at::Tensor;
void test()
{
    at::Tensor indices = at::empty({2, 3}, at::kInt);
    at::Tensor values  = at::empty({3}, at::kFloat);

    // set indices:
    // I:
    indices[0][0] = 0;
    indices[0][1] = 1;
    indices[0][2] = 2;
    // J:
    indices[1][0] = 0;
    indices[1][1] = 1;
    indices[1][2] = 2;
    // set values:
    values[0] = 1.0;
    values[1] = 2.0;
    values[2] = 3.0;

    at::Tensor sparse_tensor = at::sparse_coo_tensor(indices, values);
}
}  // namespace uipc::torch
