/**
 * @file constants.hpp
 * @brief Physical and GNSS-specific constants.
 */

#ifndef RTK_ENGINE_CONSTANTS_HPP
#define RTK_ENGINE_CONSTANTS_HPP

#include <cmath>
#include "config_manager.hpp"

namespace rtk {

inline double get_config_double(const std::string& key, double default_val) {
    return rtk_engine::ConfigManager::instance().get<double>(key) != 0.0 ? 
           rtk_engine::ConfigManager::instance().get<double>(key) : default_val;
}

/** @name Physical Constants */
///@{
inline double SPEED_OF_LIGHT() { return get_config_double("physics.speed_of_light", 299792458.0); }
///@}

/** @name GPS Frequencies */
///@{
inline double GPS_L1_FREQ() { return get_config_double("gnss.gps_l1_freq", 1575.42e6); }
inline double GPS_L2_FREQ() { return get_config_double("gnss.gps_l2_freq", 1227.60e6); }
inline double GPS_L5_FREQ() { return get_config_double("gnss.gps_l5_freq", 1176.45e6); }
inline double GPS_L1_WAVELENGTH() { return SPEED_OF_LIGHT() / GPS_L1_FREQ(); }
///@}

/** @name GLONASS Frequencies (FDMA base) */
///@{
inline double GLO_L1_BASE_FREQ() { return get_config_double("gnss.glo_l1_base_freq", 1602.0e6); }
inline double GLO_L2_BASE_FREQ() { return get_config_double("gnss.glo_l2_base_freq", 1246.0e6); }
///@}

/** @name Galileo Frequencies */
///@{
inline double GAL_E1_FREQ() { return get_config_double("gnss.gal_e1_freq", 1575.42e6); }
inline double GAL_E5B_FREQ() { return get_config_double("gnss.gal_e5b_freq", 1207.14e6); }
///@}

/** @name WGS84 Ellipsoid Constants */
///@{
inline double WGS84_A() { return get_config_double("wgs84.a", 6378137.0); }
inline double WGS84_F() { return get_config_double("wgs84.f", 1.0 / 298.257223563); }
inline double WGS84_B() { return WGS84_A() * (1.0 - WGS84_F()); }
inline double WGS84_E_SQ() { return 2.0 * WGS84_F() - WGS84_F() * WGS84_F(); }
inline double WGS84_E_PRIME_SQ() { return (WGS84_A() * WGS84_A() - WGS84_B() * WGS84_B()) / (WGS84_B() * WGS84_B()); }
///@}

} // namespace rtk

#endif // RTK_ENGINE_CONSTANTS_HPP
