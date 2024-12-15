#pragma once

namespace uipc
{
/**
 * @brief Second literal operator
 * * 
 * * ```cpp
 * * auto time = 1.0_s;
 * * ```
 */
constexpr long double operator""_s(long double value)
{
    return value;
}

/**
 * @brief Second literal operator (ms)
 */
constexpr long double operator""_ms(long double value)
{
    return value * 1e-3;
}
/**
 * @brief Meter literal operator
 * 
 * ```cpp
 * auto length = 1.0_m;
 * ```
 */
constexpr long double operator""_m(long double value)
{
    return value;
}

/**
 * @brief Meter literal operator (mm)
 */
constexpr long double operator""_mm(long double value)
{
    return value * 1e-3;
}

/**
 * @brief Meter literal operator (km)
 */
constexpr long double operator""_km(long double value)
{
    return value * 1e3;
}

/**
 * @brief Pascal literal operator
 * 
 * ```cpp
 * auto pressure = 1.0_Pa;
 * ```
 */
constexpr long double operator""_Pa(long double value)
{
    return value;
}

/**
 * @brief Pascal literal operator (kPa)
 */
constexpr long double operator""_kPa(long double value)
{
    return value * 1e3;
}

/**
 * @brief Pascal literal operator (MPa)
 */
constexpr long double operator""_MPa(long double value)
{
    return value * 1e6;
}

/**
 * @brief Pascal literal operator (GPa)
 */
constexpr long double operator""_GPa(long double value)
{
    return value * 1e9;
}

/**
 * @brief Newton literal operator
 * 
 * ```cpp
 * auto force = 1.0_N;
 * ```
 */
constexpr long double operator""_N(long double value)
{
    return value;
}
}  // namespace uipc
