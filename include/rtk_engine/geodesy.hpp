/**
 * @file geodesy.hpp
 * @brief Coordinate transformation utilities for GNSS positioning.
 */

#ifndef RTK_ENGINE_GEODESY_HPP
#define RTK_ENGINE_GEODESY_HPP

#include "rtk_engine/common.hpp"
#include <cmath>

namespace rtk {

/**
 * @brief Static utility class for WGS84 and local coordinate transforms.
 */
class Geodesy {
public:
    /**
     * @brief Convert WGS84 Geodetic to ECEF coordinates.
     * @param lat_rad Latitude in radians.
     * @param lon_rad Longitude in radians.
     * @param h_m Ellipsoidal height in meters.
     * @return Vector3 ECEF (X, Y, Z) in meters.
     */
    static Vector3 geodeticToEcef(double lat_rad, double lon_rad, double h_m) {
        double sin_lat = std::sin(lat_rad);
        double cos_lat = std::cos(lat_rad);
        double sin_lon = std::sin(lon_rad);
        double cos_lon = std::cos(lon_rad);

        double N = WGS84_A / std::sqrt(1.0 - WGS84_E_SQ * sin_lat * sin_lat);

        double X = (N + h_m) * cos_lat * cos_lon;
        double Y = (N + h_m) * cos_lat * sin_lon;
        double Z = (N * (1.0 - WGS84_E_SQ) + h_m) * sin_lat;

        return Vector3(X, Y, Z);
    }

    /**
     * @brief Convert ECEF to WGS84 Geodetic coordinates.
     * @details Uses Bowring's closed-form algorithm for millimeter-level precision.
     * @param ecef ECEF position in meters.
     * @param lat_rad Output latitude in radians.
     * @param lon_rad Output longitude in radians.
     * @param h_m Output ellipsoidal height in meters.
     */
    static void ecefToGeodetic(const Vector3& ecef, double& lat_rad, double& lon_rad, double& h_m) {
        double X = ecef.x;
        double Y = ecef.y;
        double Z = ecef.z;

        double p = std::sqrt(X * X + Y * Y);

        // Check for polar singularity
        if (p < 1e-9) {
            lon_rad = 0.0;
            if (Z > 0.0) {
                lat_rad = M_PI_2;
                h_m = Z - WGS84_B;
            } else {
                lat_rad = -M_PI_2;
                h_m = -Z - WGS84_B;
            }
            return;
        }

        double theta = std::atan2(Z * WGS84_A, p * WGS84_B);
        double sin_theta = std::sin(theta);
        double cos_theta = std::cos(theta);

        lat_rad = std::atan2(
            Z + WGS84_E_PRIME_SQ * WGS84_B * sin_theta * sin_theta * sin_theta,
            p - WGS84_E_SQ * WGS84_A * cos_theta * cos_theta * cos_theta
        );

        lon_rad = std::atan2(Y, X);

        double sin_lat = std::sin(lat_rad);
        double N = WGS84_A / std::sqrt(1.0 - WGS84_E_SQ * sin_lat * sin_lat);

        if (std::abs(lat_rad) < M_PI / 4.0) {
            h_m = p / std::cos(lat_rad) - N;
        } else {
            h_m = Z / std::sin(lat_rad) - N * (1.0 - WGS84_E_SQ);
        }
    }

    /**
     * @brief Convert ECEF to local East-North-Up (ENU) coordinates.
     * @param target_ecef ECEF position to convert.
     * @param ref_ecef ECEF position of the local ENU origin.
     * @return Vector3 ENU offset in meters.
     */
    static Vector3 ecefToEnu(const Vector3& target_ecef, const Vector3& ref_ecef) {
        double lat_ref, lon_ref, h_ref;
        ecefToGeodetic(ref_ecef, lat_ref, lon_ref, h_ref);

        Vector3 d = target_ecef - ref_ecef;

        double sin_lat = std::sin(lat_ref);
        double cos_lat = std::cos(lat_ref);
        double sin_lon = std::sin(lon_ref);
        double cos_lon = std::cos(lon_ref);

        double E = -sin_lon * d.x + cos_lon * d.y;
        double N = -sin_lat * cos_lon * d.x - sin_lat * sin_lon * d.y + cos_lat * d.z;
        double U = cos_lat * cos_lon * d.x + cos_lat * sin_lon * d.y + sin_lat * d.z;

        return Vector3(E, N, U);
    }

    /**
     * @brief Convert local ENU to ECEF coordinates.
     * @param enu ENU offset in meters.
     * @param ref_ecef ECEF position of the local ENU origin.
     * @return Vector3 Absolute ECEF position in meters.
     */
    static Vector3 enuToEcef(const Vector3& enu, const Vector3& ref_ecef) {
        double lat_ref, lon_ref, h_ref;
        ecefToGeodetic(ref_ecef, lat_ref, lon_ref, h_ref);

        double sin_lat = std::sin(lat_ref);
        double cos_lat = std::cos(lat_ref);
        double sin_lon = std::sin(lon_ref);
        double cos_lon = std::cos(lon_ref);

        double dX = -sin_lon * enu.x - sin_lat * cos_lon * enu.y + cos_lat * cos_lon * enu.z;
        double dY =  cos_lon * enu.x - sin_lat * sin_lon * enu.y + cos_lat * sin_lon * enu.z;
        double dZ =  cos_lat * enu.y + sin_lat * enu.z;

        return ref_ecef + Vector3(dX, dY, dZ);
    }
};

} // namespace rtk

#endif // RTK_ENGINE_GEODESY_HPP
