/**
 * @file klobuchar.hpp
 * @brief Implementation of the Klobuchar ionospheric delay model.
 */

#ifndef RTK_ENGINE_KLOBUCHAR_HPP
#define RTK_ENGINE_KLOBUCHAR_HPP

#include "rtk_engine/common.hpp"
#include <cmath>

namespace rtk {

/**
 * @brief Computes ionospheric path delay for single-frequency receivers.
 * @details Implements the algorithm defined in IS-GPS-200.
 */
class KlobucharModel {
public:
    /** @brief Standard default coefficients for testing. */
    static constexpr double DEFAULT_ALPHA[4] = {0.12e-7, 0.15e-7, -0.12e-6, -0.19e-6};
    static constexpr double DEFAULT_BETA[4]  = {0.98e5,  0.16e6,  -0.19e6,  -0.49e6};

    /**
     * @brief Calculate Ionospheric Path Delay in meters.
     * @param lat_deg Receiver geodetic latitude (decimal degrees).
     * @param lon_deg Receiver geodetic longitude (decimal degrees).
     * @param az_rad Satellite azimuth relative to receiver (radians).
     * @param el_rad Satellite elevation relative to receiver (radians).
     * @param gps_time_of_week Current GPS time (seconds).
     * @param alpha 4 ionospheric parameters from broadcast navigation message.
     * @param beta 4 ionospheric parameters from broadcast navigation message.
     * @return double Slant path delay in meters.
     */
    static double calculateDelay(double lat_deg, double lon_deg,
                                  double az_rad, double el_rad,
                                  double gps_time_of_week,
                                  const double alpha[4] = DEFAULT_ALPHA,
                                  const double beta[4] = DEFAULT_BETA) {
        // 1. Convert angles to semi-circles (unit of GPS broadcast parameters)
        double phi_rcv = lat_deg / 180.0;
        double lambda_rcv = lon_deg / 180.0;
        double E = el_rad / M_PI; // elevation in semi-circles

        // 2. Earth centering angle psi (semi-circles)
        double psi = 0.0137 / (E + 0.11) - 0.022;

        // 3. Sub-ionospheric latitude phi_i (semi-circles)
        double phi_i = phi_rcv + psi * std::cos(az_rad);
        
        // Strict boundary check as per IS-GPS-200
        if (phi_i > 0.416) phi_i = 0.416;
        if (phi_i < -0.416) phi_i = -0.416;

        // 4. Sub-ionospheric longitude lambda_i (semi-circles)
        double lambda_i = lambda_rcv + (psi * std::sin(az_rad)) / std::cos(phi_i * M_PI);

        // 5. Geomagnetic latitude phi_m (semi-circles)
        double phi_m = phi_i + 0.064 * std::cos((lambda_i - 1.617) * M_PI);

        // 6. Local time t_local (seconds)
        double t_local = 43200.0 * lambda_i + gps_time_of_week;
        t_local = std::fmod(t_local, 86400.0);
        if (t_local < 0.0) {
            t_local += 86400.0;
        }

        // 7. Slant factor F (obliquity factor to scale vertical delay to the line-of-sight path)
        double F = 1.0 + 16.0 * std::pow(0.5 - E, 3.0);

        // 8. Ionospheric period P (seconds)
        double P = beta[0] + beta[1] * phi_m + beta[2] * phi_m * phi_m + beta[3] * phi_m * phi_m * phi_m;
        if (P < 72000.0) {
            P = 72000.0;
        }

        // 9. Ionospheric amplitude A_amp (seconds)
        double A_amp = alpha[0] + alpha[1] * phi_m + alpha[2] * phi_m * phi_m + alpha[3] * phi_m * phi_m * phi_m;
        if (A_amp < 0.0) {
            A_amp = 0.0;
        }

        // 10. Phase X (radians)
        double X = (2.0 * M_PI * (t_local - 50400.0)) / P;

        // 11. Calculate vertical delay (seconds)
        double I_vert = 5e-9; // Night-time constant vertical delay (5 nanoseconds)
        if (std::abs(X) < 1.57) {
            // Taylor series expansion of cos(X) for efficiency on embedded systems as per GPS specifications
            double X2 = X * X;
            double X4 = X2 * X2;
            I_vert += A_amp * (1.0 - X2 / 2.0 + X4 / 24.0);
        }

        // 12. Convert slant delay in seconds to distance error in meters
        double slant_delay_sec = F * I_vert;
        return slant_delay_sec * SPEED_OF_LIGHT;
    }
};

} // namespace rtk

#endif // RTK_ENGINE_KLOBUCHAR_HPP
