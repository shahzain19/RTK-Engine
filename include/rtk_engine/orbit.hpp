/**
 * @file orbit.hpp
 * @brief Satellite orbit calculation from broadcast ephemeris.
 */

#ifndef RTK_ENGINE_ORBIT_HPP
#define RTK_ENGINE_ORBIT_HPP

#include "rtk_engine/common.hpp"
#include <cmath>

namespace rtk {

/**
 * @brief GPS/Galileo/BeiDou Broadcast Ephemeris (Keplerian Elements).
 */
struct GpsEphemeris {
    int svid;        ///< Satellite ID
    double toe;      ///< Time of Ephemeris (sec of week)
    double sqrt_a;   ///< Square root of semi-major axis (sqrt(m))
    double e;        ///< Eccentricity
    double i0;       ///< Inclination at reference time (rad)
    double omg0;     ///< Longitude of ascending node (rad)
    double omega;    ///< Argument of perigee (rad)
    double m0;       ///< Mean anomaly at reference time (rad)
    double delta_n;  ///< Mean motion difference (rad/sec)
    double idot;     ///< Rate of inclination angle (rad/sec)
    double omg_dot;  ///< Rate of right ascension (rad/sec)
    double cuc, cus; ///< Harmonic corrections (rad)
    double crc, crs; ///< Harmonic corrections (m)
    double cic, cis; ///< Harmonic corrections (rad)
    double tgd;      ///< Group delay (sec)
    double af0, af1, af2; ///< Clock biases (sec, sec/sec, sec/sec^2)
    double t_oc;     ///< Clock reference time (sec)
};

/**
 * @brief GLONASS Broadcast Ephemeris (State Vector).
 */
struct GloEphemeris {
    int svid;        ///< Satellite ID
    int freq_slot;   ///< k index (-7 to +6)
    double toe;      ///< Reference time (sec of day)
    Vector3 pos;     ///< Position in km
    Vector3 vel;     ///< Velocity in km/s
    Vector3 acc;     ///< Acceleration (Lunar/Solar perturbations) in km/s^2
    double tau_n;    ///< Clock bias (sec)
    double gamma_n;  ///< Relative freq bias
};

/**
 * @brief Computes precise satellite ECEF positions and clock offsets.
 */
class SatelliteOrbit {
public:
    /** @name Geocentric Gravitational Constants */
    ///@{
    static constexpr double GM_GPS = 3.986005e14;
    static constexpr double GM_GAL = 3.986004418e14;
    static constexpr double GM_BDS = 3.986004418e14;
    ///@}

    /** @name Earth Rotation Rates */
    ///@{
    static constexpr double OMEGA_GPS = 7.2921151467e-5;
    static constexpr double OMEGA_GAL = 7.2921151467e-5;
    static constexpr double OMEGA_BDS = 7.292115e-5;
    ///@}

    /**
     * @brief Compute satellite position for Keplerian-based systems.
     * @param eph Keplerian ephemeris data.
     * @param t Target GPS time (seconds).
     * @param dt_s Output satellite clock bias (seconds).
     * @param sys Target constellation (GPS/GAL/BDS).
     * @return Vector3 Satellite ECEF position in meters.
     */
    static Vector3 computePosition(const GpsEphemeris& eph, double t, double& dt_s, Constellation sys = Constellation::GPS) {
        double gm = GM_GPS;
        double omega_e = OMEGA_GPS;

        if (sys == Constellation::GALILEO) {
            gm = GM_GAL;
            omega_e = OMEGA_GAL;
        } else if (sys == Constellation::BEIDOU) {
            gm = GM_BDS;
            omega_e = OMEGA_BDS;
        }

        // 1. Time from reference epoch
        double tk = t - eph.toe;
        if (tk > 302400.0) tk -= 604800.0;
        if (tk < -302400.0) tk += 604800.0;

        // 2. Mean motion and mean anomaly
        double a = eph.sqrt_a * eph.sqrt_a;
        double n0 = std::sqrt(gm / (a * a * a));
        double n = n0 + eph.delta_n;
        double mk = eph.m0 + n * tk;
        
        // 3. Solve Kepler's equation for Eccentric Anomaly (ek)
        double ek = mk;
        for (int i = 0; i < 10; ++i) {
            double ek_old = ek;
            ek = mk + eph.e * std::sin(ek);
            if (std::abs(ek - ek_old) < 1e-12) break;
        }

        // 4. True anomaly and argument of latitude
        double cos_ek = std::cos(ek);
        double sin_ek = std::sin(ek);
        double vk = std::atan2(std::sqrt(1.0 - eph.e * eph.e) * sin_ek, cos_ek - eph.e);
        double phi = vk + eph.omega;

        // 5. Harmonic corrections
        double du = eph.cus * std::sin(2.0 * phi) + eph.cuc * std::cos(2.0 * phi);
        double dr = eph.crs * std::sin(2.0 * phi) + eph.crc * std::cos(2.0 * phi);
        double di = eph.cis * std::sin(2.0 * phi) + eph.cic * std::cos(2.0 * phi);

        // 6. Corrected orbital parameters
        double u = phi + du;
        double r = a * (1.0 - eph.e * cos_ek) + dr;
        double i = eph.i0 + di + eph.idot * tk;

        // 7. Position in orbital plane
        double x_prime = r * std::cos(u);
        double y_prime = r * std::sin(u);

        // 8. Longitude of ascending node
        double omg = eph.omg0 + (eph.omg_dot - omega_e) * tk - omega_e * eph.toe;

        double cos_omg = std::cos(omg);
        double sin_omg = std::sin(omg);
        double cos_i = std::cos(i);
        double sin_i = std::sin(i);

        // 9. ECEF coordinates
        double x = x_prime * cos_omg - y_prime * cos_i * sin_omg;
        double y = x_prime * sin_omg + y_prime * cos_i * cos_omg;
        double z = y_prime * sin_i;

        // 10. Satellite clock bias
        double tk_clock = t - eph.t_oc;
        if (tk_clock > 302400.0) tk_clock -= 604800.0;
        if (tk_clock < -302400.0) tk_clock += 604800.0;
        dt_s = eph.af0 + eph.af1 * tk_clock + eph.af2 * tk_clock * tk_clock;
        
        // Relativistic effect
        dt_s += (2.0 * std::sqrt(gm * a) / (SPEED_OF_LIGHT() * SPEED_OF_LIGHT())) * eph.e * sin_ek;
        dt_s -= eph.tgd;

        return Vector3(x, y, z);
    }

    /**
     * @brief Integrates GLONASS differential equations (RK4).
     * @param eph GLONASS state vector ephemeris.
     * @param t Target time (seconds of day).
     * @param dt_s Output satellite clock bias (seconds).
     * @return Vector3 Satellite ECEF position in meters.
     */
    static Vector3 computeGlonassPosition(const GloEphemeris& eph, double t, double& dt_s) {
        double tk = t - eph.toe;
        if (tk > 43200.0) tk -= 86400.0;
        if (tk < -43200.0) tk += 86400.0;

        State s = {eph.pos, eph.vel};
        double h = 60.0; // 60 second step size for RK4
        int total_steps = static_cast<int>(std::abs(tk) / h);
        double last_h = std::fmod(std::abs(tk), h);
        double step_dir = (tk < 0) ? -1.0 : 1.0;

        for (int i = 0; i <= total_steps; ++i) {
            double step_h = (i == total_steps) ? (step_dir * last_h) : (step_dir * h);
            if (std::abs(step_h) < 1e-9) break;

            State k1 = glonassDiffEq(s, eph.acc);
            State s2 = {s.r + k1.r * (step_h * 0.5), s.v + k1.v * (step_h * 0.5)};
            State k2 = glonassDiffEq(s2, eph.acc);
            State s3 = {s.r + k2.r * (step_h * 0.5), s.v + k2.v * (step_h * 0.5)};
            State k3 = glonassDiffEq(s3, eph.acc);
            State s4 = {s.r + k3.r * step_h, s.v + k3.v * step_h};
            State k4 = glonassDiffEq(s4, eph.acc);

            s.r = s.r + (k1.r + k2.r * 2.0 + k3.r * 2.0 + k4.r) * (step_h / 6.0);
            s.v = s.v + (k1.v + k2.v * 2.0 + k3.v * 2.0 + k4.v) * (step_h / 6.0);
        }

        dt_s = -eph.tau_n + eph.gamma_n * tk;
        return s.r * 1000.0; // convert km to meters
    }

private:
    struct State {
        Vector3 r; // km
        Vector3 v; // km/s
    };

    /** @brief GLONASS EOM (Equations of Motion) with J2 perturbations. */
    static State glonassDiffEq(const State& s, const Vector3& acc_p) {
        constexpr double MU = 398600.44;     // km^3/s^2
        constexpr double OMEGA = 7.292115e-5; // rad/s
        constexpr double RE = 6378.136;      // km
        constexpr double J2 = 1.0826257e-3;

        double r2 = s.r.x * s.r.x + s.r.y * s.r.y + s.r.z * s.r.z;
        double r = std::sqrt(r2);
        double r3 = r2 * r;
        double r5 = r2 * r2 * r;

        double z2_r2 = (s.r.z * s.r.z) / r2;
        double coeff = 1.5 * J2 * MU * (RE * RE) / r5;

        State ds;
        ds.r = s.v;
        ds.v.x = -MU * s.r.x / r3 + coeff * s.r.x * (1.0 - 5.0 * z2_r2) + OMEGA * OMEGA * s.r.x + 2.0 * OMEGA * s.v.y + acc_p.x;
        ds.v.y = -MU * s.r.y / r3 + coeff * s.r.y * (1.0 - 5.0 * z2_r2) + OMEGA * OMEGA * s.r.y - 2.0 * OMEGA * s.v.x + acc_p.y;
        ds.v.z = -MU * s.r.z / r3 + coeff * s.r.z * (3.0 - 5.0 * z2_r2) + acc_p.z;

        return ds;
    }
};

} // namespace rtk

#endif // RTK_ENGINE_ORBIT_HPP
