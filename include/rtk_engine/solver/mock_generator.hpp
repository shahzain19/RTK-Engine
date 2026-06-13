/**
 * @file mock_generator.hpp
 * @brief Synthetic GNSS observation generator for testing Multi-GNSS and Kinematic motion.
 */

#ifndef RTK_ENGINE_MOCK_GENERATOR_HPP
#define RTK_ENGINE_MOCK_GENERATOR_HPP

#include "rtk_engine/common.hpp"
#include "rtk_engine/geodesy.hpp"
#include "rtk_engine/klobuchar.hpp"
#include <vector>
#include <random>

namespace rtk {

/**
 * @brief Generates high-fidelity synthetic GNSS observations with errors.
 */
class MockGenerator {
public:
    static double generateGaussianNoise(double mean, double stddev) {
        static std::mt19937 gen(1337);
        std::normal_distribution<double> dist(mean, stddev);
        return dist(gen);
    }

    static void generateMockObservations(const Vector3& true_base_ecef,
                                         const Vector3& true_rover_ecef,
                                         double gps_time,
                                         EpochObs& base_obs,
                                         EpochObs& rover_obs) {
        base_obs.gps_time = gps_time;
        base_obs.station_id = 1001; 
        base_obs.ref_pos = true_base_ecef;

        rover_obs.gps_time = gps_time;
        rover_obs.station_id = 2002; 
        rover_obs.ref_pos = Vector3(0,0,0);

        struct MockSatConfig {
            int svid;
            Constellation sys;
            double elevation_deg;
            double azimuth_deg;
            int ambiguity_cycles_l1;
            int ambiguity_cycles_l2;
        };

        const std::vector<MockSatConfig> sat_configs = {
            {1,  Constellation::GPS, 55.0, 10.0,   1200, 900},
            {3,  Constellation::GPS, 65.0, 45.0,  15243, 11000}, 
            {8,  Constellation::GPS, 42.0, 135.0, -8432, -6000},
            {14, Constellation::GPS, 28.0, 225.0,  29481, 21000},
            {22, Constellation::GPS, 55.0, 315.0,  1029, 800},
            {2,  Constellation::GALILEO, 55.0, 315.0, 1029, 800},
            {12, Constellation::GALILEO, 78.0, 270.0, 5829, 4500},
            {19, Constellation::GALILEO, 35.0, 180.0, -2000, -1500},
            {24, Constellation::GALILEO, 45.0, 60.0,  3000, 2500},
            {5,  Constellation::GLONASS, 18.0, 90.0, -39201, -30000},
            {10, Constellation::GLONASS, 60.0, 200.0, 4000, 3000},
            {15, Constellation::GLONASS, 30.0, 300.0, -1000, -800}
        };

        double lat_b_rad, lon_b_rad, h_b;
        Geodesy::ecefToGeodetic(true_base_ecef, lat_b_rad, lon_b_rad, h_b);
        double lat_r_rad, lon_r_rad, h_r;
        Geodesy::ecefToGeodetic(true_rover_ecef, lat_r_rad, lon_r_rad, h_r);

        for (const auto& config : sat_configs) {
            double el_b_rad = config.elevation_deg * M_PI / 180.0;
            double az_b_rad = config.azimuth_deg * M_PI / 180.0;

            double range_b_approx = 20200000.0; 
            if (config.sys == Constellation::GALILEO) range_b_approx = 23222000.0;
            if (config.sys == Constellation::GLONASS) range_b_approx = 19100000.0;

            Vector3 u_enu_b(std::cos(el_b_rad) * std::sin(az_b_rad), std::cos(el_b_rad) * std::cos(az_b_rad), std::sin(el_b_rad));
            Vector3 u_ecef = Geodesy::enuToEcef(u_enu_b, true_base_ecef) - true_base_ecef;
            u_ecef = u_ecef.normalized();
            Vector3 sat_ecef = true_base_ecef + u_ecef * range_b_approx;

            double rho_b = (sat_ecef - true_base_ecef).norm();
            double rho_r = (sat_ecef - true_rover_ecef).norm();

            Vector3 u_enu_r = Geodesy::ecefToEnu(sat_ecef, true_rover_ecef);
            double el_r_rad = std::atan2(u_enu_r.z, std::sqrt(u_enu_r.x * u_enu_r.x + u_enu_r.y * u_enu_r.y));
            double az_r_rad = std::atan2(u_enu_r.x, u_enu_r.y);
            if (az_r_rad < 0) az_r_rad += 2.0 * M_PI;

            double iono_b = KlobucharModel::calculateDelay(lat_b_rad * 180 / M_PI, lon_b_rad * 180 / M_PI, az_b_rad, el_b_rad, gps_time);
            double iono_r = KlobucharModel::calculateDelay(lat_r_rad * 180 / M_PI, lon_r_rad * 180 / M_PI, az_r_rad, el_r_rad, gps_time);
            double tropo_b = 2.31 / (std::sin(el_b_rad) + 0.05);
            double tropo_r = 2.31 / (std::sin(el_r_rad) + 0.05);

            double f1 = GPS_L1_FREQ, f2 = GPS_L2_FREQ;
            if (config.sys == Constellation::GALILEO) { f1 = GAL_E1_FREQ; f2 = GAL_E5B_FREQ; }
            else if (config.sys == Constellation::GLONASS) { f1 = GLO_L1_BASE_FREQ; f2 = GLO_L2_BASE_FREQ; }
            double lam1 = SPEED_OF_LIGHT / f1;
            double lam2 = SPEED_OF_LIGHT / f2;

            SatelliteObs b_sat; b_sat.svid = config.svid; b_sat.sys = config.sys; b_sat.elevation = el_b_rad; b_sat.azimuth = az_b_rad;
            SignalObs b1, b2;
            b1.frequency = f1; b1.pseudorange = rho_b + iono_b + tropo_b + generateGaussianNoise(0, 0.01);
            b1.carrier_phase = (rho_b - iono_b + tropo_b + generateGaussianNoise(0, 0.001)) / lam1;
            b2.frequency = f2; double i2_b = iono_b * (f1*f1)/(f2*f2);
            b2.pseudorange = rho_b + i2_b + tropo_b + generateGaussianNoise(0, 0.01);
            b2.carrier_phase = (rho_b - i2_b + tropo_b + generateGaussianNoise(0, 0.001)) / lam2;
            b_sat.signals = {b1, b2}; b_sat.pseudorange = b1.pseudorange; b_sat.carrier_phase = b1.carrier_phase;
            base_obs.sat_obs.push_back(b_sat);

            SatelliteObs r_sat; r_sat.svid = config.svid; r_sat.sys = config.sys; r_sat.elevation = el_r_rad; r_sat.azimuth = az_r_rad;
            SignalObs r1, r2;
            r1.frequency = f1; r1.pseudorange = rho_r + iono_r + tropo_r + generateGaussianNoise(0, 0.02);
            r1.carrier_phase = (rho_r - iono_r + tropo_r + generateGaussianNoise(0, 0.002)) / lam1 + config.ambiguity_cycles_l1;
            r2.frequency = f2; double i2_r = iono_r * (f1*f1)/(f2*f2);
            r2.pseudorange = rho_r + i2_r + tropo_r + generateGaussianNoise(0, 0.02);
            r2.carrier_phase = (rho_r - i2_r + tropo_r + generateGaussianNoise(0, 0.002)) / lam2 + config.ambiguity_cycles_l2;
            r_sat.signals = {r1, r2}; r_sat.pseudorange = r1.pseudorange; r_sat.carrier_phase = r1.carrier_phase;
            rover_obs.sat_obs.push_back(r_sat);
        }
    }

    /**
     * @brief Pre-calculates satellite positions for a set of configs once to save time in loops.
     */
    static std::vector<Vector3> precomputeSatPositions(const Vector3& true_base_ecef) {
        const std::vector<double> el_degs = {55.0, 65.0, 42.0, 28.0, 55.0, 55.0, 78.0, 35.0, 45.0, 18.0, 60.0, 30.0};
        const std::vector<double> az_degs = {10.0, 45.0, 135.0, 225.0, 315.0, 315.0, 270.0, 180.0, 60.0, 90.0, 200.0, 300.0};
        const std::vector<double> ranges  = {20.2e6, 20.2e6, 20.2e6, 20.2e6, 20.2e6, 23.2e6, 23.2e6, 23.2e6, 23.2e6, 19.1e6, 19.1e6, 19.1e6};
        
        std::vector<Vector3> pos;
        for (size_t i = 0; i < el_degs.size(); ++i) {
            double el = el_degs[i] * M_PI / 180.0, az = az_degs[i] * M_PI / 180.0;
            Vector3 u_enu(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el));
            Vector3 u_ecef = (Geodesy::enuToEcef(u_enu, true_base_ecef) - true_base_ecef).normalized();
            pos.push_back(true_base_ecef + u_ecef * ranges[i]);
        }
        return pos;
    }
};

} // namespace rtk

#endif // RTK_ENGINE_MOCK_GENERATOR_HPP
