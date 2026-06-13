/**
 * @file constants.hpp
 * @brief Physical and GNSS-specific constants.
 */

#ifndef RTK_ENGINE_CONSTANTS_HPP
#define RTK_ENGINE_CONSTANTS_HPP

#include <cmath>

namespace rtk {

/** @name Physical Constants */
///@{
constexpr double SPEED_OF_LIGHT = 299792458.0; ///< Speed of light in vacuum (m/s)
///@}

/** @name GPS Frequencies */
///@{
constexpr double GPS_L1_FREQ = 1575.42e6;      ///< GPS L1 carrier frequency (Hz)
constexpr double GPS_L2_FREQ = 1227.60e6;      ///< GPS L2 carrier frequency (Hz)
constexpr double GPS_L5_FREQ = 1176.45e6;      ///< GPS L5 carrier frequency (Hz)
constexpr double GPS_L1_WAVELENGTH = SPEED_OF_LIGHT / GPS_L1_FREQ; ///< ~0.19029 m
///@}

/** @name GLONASS Frequencies (FDMA base) */
///@{
constexpr double GLO_L1_BASE_FREQ = 1602.0e6;    ///< GLONASS L1 base frequency (Hz)
constexpr double GLO_L1_DELTA_FREQ = 0.5625e6;   ///< GLONASS L1 channel spacing (Hz)
constexpr double GLO_L2_BASE_FREQ = 1246.0e6;    ///< GLONASS L2 base frequency (Hz)
constexpr double GLO_L2_DELTA_FREQ = 0.4375e6;   ///< GLONASS L2 channel spacing (Hz)
///@}

/** @name Galileo Frequencies */
///@{
constexpr double GAL_E1_FREQ = 1575.42e6;      ///< Galileo E1 (same as GPS L1)
constexpr double GAL_E5A_FREQ = 1176.45e6;     ///< Galileo E5a (same as GPS L5)
constexpr double GAL_E5B_FREQ = 1207.14e6;     ///< Galileo E5b (Hz)
constexpr double GAL_E6_FREQ = 1278.75e6;      ///< Galileo E6 (Hz)
///@}

/** @name BeiDou Frequencies */
///@{
constexpr double BDS_B1_FREQ = 1561.098e6;     ///< BeiDou B1I frequency (Hz)
constexpr double BDS_B2_FREQ = 1207.14e6;      ///< BeiDou B2I frequency (Hz)
constexpr double BDS_B3_FREQ = 1268.52e6;      ///< BeiDou B3I frequency (Hz)
///@}

/** @name WGS84 Ellipsoid Constants */
///@{
constexpr double WGS84_A = 6378137.0;             ///< Semi-major axis (meters)
constexpr double WGS84_F = 1.0 / 298.257223563;  ///< Flattening
constexpr double WGS84_B = WGS84_A * (1.0 - WGS84_F); ///< Semi-minor axis
constexpr double WGS84_E_SQ = 2.0 * WGS84_F - WGS84_F * WGS84_F; ///< First eccentricity squared
constexpr double WGS84_E_PRIME_SQ = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B); ///< Second eccentricity squared
///@}

} // namespace rtk

#endif // RTK_ENGINE_CONSTANTS_HPP
