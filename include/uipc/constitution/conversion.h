#pragma once
#include <uipc/common/type_define.h>

namespace uipc::constitution
{
inline void EG_to_lame(Float E, Float G, Float& lambda, Float& mu, Float& poisson)
{
    // ref: https://en.wikipedia.org/wiki/Lam%C3%A9_parameters

    //tex: $\mu = G$
    mu = G;
    //tex: $\lambda = \frac{G(E-2 G)}{3 G-E}$
    lambda = G * (E - 2 * G) / (3 * G - E);

    //tex: $\nu = {\frac {E}{2G}}-1$
    poisson = E / (2 * G) - 1;
}

inline void lame_to_EG(Float lambda, Float mu, Float& E, Float& G, Float& poisson)
{
    // ref: https://en.wikipedia.org/wiki/Lam%C3%A9_parameters

    //tex: $G = \mu$
    G = mu;
    //tex: $ E = \frac{G(3 \lambda+2 G)}{\lambda+G}$
    E = G * (3 * lambda + 2 * G) / (lambda + G);
    //tex: $\nu = \frac{\lambda}{2(\lambda+G)}$
    poisson = lambda / (2 * (lambda + G));
}

inline void lame_to_poisson(Float lambda, Float mu, Float& poisson)
{
    //tex: $\nu = \frac{\lambda}{2(\lambda+\mu)}$
    poisson = lambda / (2 * (lambda + mu));
}

inline void EG_to_poisson(Float E, Float G, Float& poisson)
{
    //tex: $\nu = {\frac {E}{2G}}-1$
    poisson = E / (2 * G) - 1;
}

inline void EP_to_lame(Float E, Float poission, Float& lambda, Float& mu)
{
    lambda = E * poission / (1 + poission) / (1 - 2 * poission);
    mu     = E / (2 * (1 + poission));
}
}  // namespace uipc::constitution
