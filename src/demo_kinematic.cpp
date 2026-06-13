/**
 * @file demo_kinematic.cpp
 * @brief Demonstration of the Extended Kalman Filter (EKF) for kinematic tracking.
 */

#include "rtk_engine/common.hpp"
#include "rtk_engine/geodesy.hpp"
#include "rtk_engine/solver/ekf_filter.hpp"
#include "rtk_engine/solver/mock_generator.hpp"
#include "rtk_engine/rtk_solver.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

int main() {
    std::cout << "========================================================================\n";
    std::cout << "        PHASE 4: KINEMATIC TRACKING WITH EXTENDED KALMAN FILTER          \n";
    std::cout << "========================================================================\n\n";

    // 1. SETUP BASE STATION
    rtk::Vector3 base_ecef = rtk::Geodesy::geodeticToEcef(37.422 * M_PI / 180.0, -122.084 * M_PI / 180.0, 30.0);
    auto sat_positions = rtk::MockGenerator::precomputeSatPositions(base_ecef);

    // 2. INITIALIZE EKF
    rtk::EkfFilter ekf;
    double t = 0.0;
    
    // Initial guess at TRUE position of first epoch to avoid transient
    rtk::Vector3 true_start_enu(20.0, 0.0, 0.0);
    rtk::Vector3 true_start_ecef = rtk::Geodesy::enuToEcef(true_start_enu, base_ecef);
    ekf.initialize(true_start_ecef, t);

    std::cout << "[EKF] Initialized at Rover start coordinates.\n";
    std::cout << "------------------------------------------------------------------------\n";
    std::cout << " TIME (s) | TRUE ENU (E,N) | EKF ERROR (m) | DGPS ERROR (m) | STATUS   \n";
    std::cout << "------------------------------------------------------------------------\n";

    // 3. KINEMATIC LOOP (60 Epochs)
    for (int epoch = 0; epoch < 60; ++epoch) {
        t += 1.0;
        
        // Circular trajectory: R=20m, 5 deg/sec
        double angle = (epoch * 5.0) * M_PI / 180.0;
        rtk::Vector3 true_enu(20.0 * std::cos(angle), 20.0 * std::sin(angle), 0.0);
        rtk::Vector3 true_ecef = rtk::Geodesy::enuToEcef(true_enu, base_ecef);

        // Generate Observations
        rtk::EpochObs base_obs, rover_obs;
        rtk::MockGenerator::generateMockObservations(base_ecef, true_ecef, t, base_obs, rover_obs);

        // Simulate a cycle slip on SVID 3 at epoch 30
        if (epoch == 30) {
            for (auto& obs : rover_obs.sat_obs) {
                if (obs.svid == 3) {
                    obs.carrier_phase += 10.0; // 10 cycle jump
                    for(auto& s : obs.signals) s.carrier_phase += 10.0;
                }
            }
        }

        // --- SOLVER A: Standard DGPS (Least Squares) ---
        rtk::Vector3 dgps_ecef;
        rtk::RtkSolver::solvePositionDgps(base_obs, rover_obs, true_ecef, dgps_ecef);
        double dgps_err = (rtk::Geodesy::ecefToEnu(dgps_ecef, base_ecef) - true_enu).norm();

        // --- SOLVER B: EKF (Continuous Tracking) ---
        ekf.predict(t);
        ekf.update(base_obs, rover_obs, sat_positions);
        
        rtk::Vector3 ekf_ecef(ekf.getState()(0), ekf.getState()(1), ekf.getState()(2));
        double ekf_err = (rtk::Geodesy::ecefToEnu(ekf_ecef, base_ecef) - true_enu).norm();

        std::string status = (epoch == 30) ? "SLIP!" : "";
        
        std::cout << std::setw(8) << t << " | " 
                  << std::fixed << std::setprecision(2)
                  << std::setw(6) << true_enu.x << "," << std::setw(6) << true_enu.y << " | "
                  << std::setw(13) << ekf_err << " | "
                  << std::setw(14) << dgps_err << " | "
                  << status << "\n";
    }

    std::cout << "------------------------------------------------------------------------\n";
    std::cout << "EKF tracking completed.\n";
    return 0;
}
