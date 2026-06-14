#pragma once

namespace rtk {

class TroposphereModel {
public:
    /**
     * @brief Saastamoinen tropospheric delay model.
     * @param lat_rad Latitude (radians).
     * @param h_m Ellipsoidal height (meters).
     * @param el_rad Elevation angle (radians).
     * @return Tropospheric delay in meters.
     */
    static double getDelay(double lat_rad, double h_m, double el_rad);
};

} // namespace rtk
