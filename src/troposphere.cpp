#include "rtk_engine/troposphere.hpp"
#include <cmath>

namespace rtk {

double TroposphereModel::getDelay(double lat_rad, double h_m, double el_rad) {
    if (el_rad <= 0.0) return 0.0;
    
    // Simplified Saastamoinen model
    double P = 1013.25; // Pressure (hPa)
    double T = 20.0;    // Temp (C)
    
    double zenith_delay = 0.002277 * P / std::cos(el_rad);
    return zenith_delay;
}

} // namespace rtk
