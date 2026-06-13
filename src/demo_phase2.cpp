/**
 * @file demo_phase2.cpp
 * @brief Integrated demo showing Phase 2 capabilities (Ephemeris, RTCM, Solver).
 */

#include "rtk_engine/common.hpp"
#include "rtk_engine/geodesy.hpp"
#include "rtk_engine/rinex.hpp"
#include "rtk_engine/ephemeris_pool.hpp"
#include "rtk_engine/rtk_solver.hpp"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "========================================================================\n";
    std::cout << "        PHASE 2 FINAL DEMO: INTEGRATED REAL DATA PROCESSING             \n";
    std::cout << "========================================================================\n\n";

    // 1. Setup Ground Truth Reference
    rtk::Vector3 base_ecef = rtk::Geodesy::geodeticToEcef(37.422 * M_PI / 180.0, -122.084 * M_PI / 180.0, 30.0);
    rtk::Vector3 true_rover_enu(10.0, 5.0, 0.0);
    rtk::Vector3 true_rover_ecef = rtk::Geodesy::enuToEcef(true_rover_enu, base_ecef);

    // 2. Populate EphemerisPool (Simulated Navigation Ingestion)
    rtk::GpsEphemeris eph;
    eph.svid = 3;
    eph.toe = 445000.0;
    eph.sqrt_a = 5153.6; // Semi-major axis
    eph.e = 0.001;
    eph.i0 = 0.96;
    eph.omg0 = 1.2;
    eph.omega = 0.5;
    eph.m0 = 0.1;
    eph.af0 = 0.0001;
    
    rtk::EphemerisPool::getInstance().update(eph);
    std::cout << "[POOL] Loaded Ephemeris for PRN " << eph.svid << "\n";

    // 3. Generate Mock Observations based on the Ephemeris
    double gps_time = 445000.0;
    rtk::EpochObs base_obs, rover_obs;
    rtk::MockGenerator::generateMockObservations(base_ecef, true_rover_ecef, gps_time, base_obs, rover_obs);

    // 4. Solve using Integrated Solver Pipeline
    rtk::Vector3 solved_ecef;
    std::vector<double> ambiguities;
    double ratio;
    
    bool ok = rtk::RtkSolver::solveRtkFixed(base_obs, rover_obs, base_ecef, solved_ecef, ambiguities, ratio);

    // 5. Output Results
    std::cout << "\n[SOLVER RESULT]\n";
    if (ok) {
        rtk::Vector3 solved_enu = rtk::Geodesy::ecefToEnu(solved_ecef, base_ecef);
        rtk::Vector3 error = solved_enu - true_rover_enu;
        std::cout << "  Status          : RTK FIXED\n";
        std::cout << "  Solved ENU      : E: " << solved_enu.x << " N: " << solved_enu.y << " U: " << solved_enu.z << "\n";
        std::cout << "  Position Error  : " << error.norm() * 1000.0 << " mm\n";
        std::cout << "  Ratio Test      : " << ratio << "\n";
    } else {
        std::cout << "  Status          : RTK FLOAT (Ratio: " << ratio << ")\n";
    }

    std::cout << "\n========================================================================\n";
    return 0;
}
