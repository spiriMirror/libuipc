#pragma once
#include <type_define.h>
#include <muda/muda_def.h>

namespace uipc::backend::cuda
{
//tex:
//$$ S = \frac{V_{\perp}}{\kappa v} $$
//we don't include the $\kappa$ and $v$ calculate it by yourself and multiply it

//tex:
//$$
// \frac{V_{\perp}}{\kappa v} =\sum\left(a_{i} \cdot a_{i}-1\right)^{2}
// +\sum_{i \neq j}\left(a_{i} \cdot a_{j}\right)^{2}
//$$
MUDA_GENERIC Float shape_energy(const Vector12& q);

//tex:
// $$\frac{1}{\kappa v}\frac{\partial V_{\perp}}{\partial a_{i}}=
//2 \left(2\left(a_{i} \cdot a_{i}-1\right) a_{i}
//+ \sum a_{j}  (a_{j} \cdot a_{i})\right)$$
MUDA_GENERIC Vector9 shape_energy_gradient(const Vector12& q);


MUDA_GENERIC Matrix9x9 shape_energy_hessian(const Vector12& q);
}  // namespace uipc::backend::cuda