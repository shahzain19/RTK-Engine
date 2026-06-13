/**
 * @file main.cpp
 * @brief Multi-GNSS & Multi-Frequency RTK demonstration.
 */

#include "rtk_engine/common.hpp"
#include "rtk_engine/geodesy.hpp"
#include "rtk_engine/nmea.hpp"
#include "rtk_engine/klobuchar.hpp"
#include "rtk_engine/rtk_solver.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <cmath>

int main() {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "========================================================================\n";
    std::cout << "        PHASE 3: MULTI-GNSS & MULTI-FREQUENCY RTK ENGINE                \n";
    std::cout << "========================================================================\n\n";

    // 1. SETUP GROUND TRUTH
    const double base_lat = 37.422075 * M_PI / 180.0;
    const double base_lon = -122.084025 * M_PI / 180.0;
    const double base_h = 30.0;
    rtk::Vector3 base_ecef = rtk::Geodesy::geodeticToEcef(base_lat, base_lon, base_h);

    // True Rover position (located 600m East, 800m North)
    rtk::Vector3 true_rover_enu(600.0, 800.0, 0.0);
    rtk::Vector3 true_rover_ecef = rtk::Geodesy::enuToEcef(true_rover_enu, base_ecef);

    std::cout << "[SETUP] Base: Apple Park, Rover: 1km away.\n";
    std::cout << "[SETUP] Tracking: GPS, GLONASS, Galileo (L1/L2, E1/E5b)\n\n";

    // 2. GENERATE MULTI-GNSS MOCK OBSERVATIONS
    double gps_time = 172814.00;
    rtk::EpochObs base_obs, rover_obs;
    rtk::MockGenerator::generateMockObservations(base_ecef, true_rover_ecef, gps_time, base_obs, rover_obs);

    // SIMULATE A FAULTY SATELLITE (PRN 8) - Add 50m error to test RAIM
    for (auto& obs : rover_obs.sat_obs) {
        if (obs.svid == 8 && obs.sys == rtk::Constellation::GPS) {
            std::cout << "[SIM] Adding 50m outlier to GPS PRN 8 for RAIM testing...\n";
            obs.pseudorange += 50.0;
            for (auto& sig : obs.signals) sig.pseudorange += 50.0;
        }
    }

    // Initial guess from NMEA (simulate 10m error)
    rtk::Vector3 initial_rover_ecef = true_rover_ecef + rtk::Vector3(5.0, -8.0, 4.0);

    // 3. COMPARE DIFFERENT SOLVER MODES
    
    // Mode A: Single-Frequency (L1), No Iono Corrections
    rtk::Vector3 res_a;
    bool ok_a = rtk::RtkSolver::solvePositionDgps(base_obs, rover_obs, initial_rover_ecef, res_a, false, false);
    double err_a = (rtk::Geodesy::ecefToEnu(res_a, base_ecef) - true_rover_enu).norm();

    // Mode B: Single-Frequency (L1), With Klobuchar Corrections
    rtk::Vector3 res_b;
    bool ok_b = rtk::RtkSolver::solvePositionDgps(base_obs, rover_obs, initial_rover_ecef, res_b, true, false);
    double err_b = (rtk::Geodesy::ecefToEnu(res_b, base_ecef) - true_rover_enu).norm();

    // Mode C: Multi-Frequency, Ionosphere-Free (IF) Combination
    rtk::Vector3 res_c;
    bool ok_c = rtk::RtkSolver::solvePositionDgps(base_obs, rover_obs, initial_rover_ecef, res_c, false, true);
    double err_c = (rtk::Geodesy::ecefToEnu(res_c, base_ecef) - true_rover_enu).norm();

    std::cout << "------------------------------------------------------------------------\n";
    std::cout << "  SOLVER MODE                          | CONVERGED | 3D ERROR (m)      \n";
    std::cout << "------------------------------------------------------------------------\n";
    std::cout << "  Single-Freq L1 (No Iono Corr)        | " << (ok_a ? "   YES   " : "   NO    ") << " | " << std::setw(10) << err_a << "\n";
    std::cout << "  Single-Freq L1 (With Klobuchar)      | " << (ok_b ? "   YES   " : "   NO    ") << " | " << std::setw(10) << err_b << "\n";
    std::cout << "  Multi-Freq Multi-GNSS (IF-DGPS)      | " << (ok_c ? "   YES   " : "   NO    ") << " | " << std::setw(10) << err_c << "\n";
    std::cout << "------------------------------------------------------------------------\n\n";

    // 4. RTK FIXED (LAMBDA) - Future Phase 3 Enhancement
    rtk::Vector3 res_fixed;
    std::vector<double> ambs;
    double ratio;
    bool ok_fixed = rtk::RtkSolver::solveRtkFixed(base_obs, rover_obs, res_c, res_fixed, ambs, ratio);

    if (ok_fixed) {
        double err_fixed = (rtk::Geodesy::ecefToEnu(res_fixed, base_ecef) - true_rover_enu).norm();
        std::cout << "[RESULT] RTK Fixed Status: SUCCESS (Ratio: " << ratio << ")\n";
        std::cout << "[RESULT] Final Precision: " << (err_fixed * 100.0) << " cm\n";
    } else {
        std::cout << "[RESULT] RTK Fixed Status: FLOAT (Ratio: " << ratio << ")\n";
    }

    std::cout << "\n========================================================================\n";
    return 0;
}
