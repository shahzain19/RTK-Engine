/**
 * @file rtk_engine_pro.cpp
 * @brief Final integrated production-ready RTK engine demonstration with EKF and Dashboard.
 */

#include "rtk_engine/common.hpp"
#include "rtk_engine/geodesy.hpp"
#include "rtk_engine/solver/ekf_filter.hpp"
#include "rtk_engine/solver/mock_generator.hpp"
#include "rtk_engine/ui/dashboard.hpp"
#include <chrono>
#include <thread>
#include <vector>

int main() {
    rtk::Dashboard db;
    db.reset();

    // 1. SETUP
    rtk::Vector3 base_ecef = rtk::Geodesy::geodeticToEcef(37.422 * M_PI / 180.0, -122.084 * M_PI / 180.0, 30.0);
    auto sat_positions = rtk::MockGenerator::precomputeSatPositions(base_ecef);

    rtk::EkfFilter ekf;
    double t = 0.0;
    
    rtk::Vector3 true_start_enu(20.0, 0.0, 0.0);
    rtk::Vector3 true_start_ecef = rtk::Geodesy::enuToEcef(true_start_enu, base_ecef);
    ekf.initialize(true_start_ecef, t);

    // 2. KINEMATIC SIMULATION (100 Epochs at 10Hz)
    // Commercial grade requires high-frequency processing
    for (int epoch = 0; epoch < 100; ++epoch) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        t += 0.1; // 10Hz updates
        
        // Dynamic circular motion
        double angle = (t * 10.0) * M_PI / 180.0;
        rtk::Vector3 true_enu(15.0 * std::cos(angle), 15.0 * std::sin(angle), 0.0);
        rtk::Vector3 true_ecef = rtk::Geodesy::enuToEcef(true_enu, base_ecef);
        
        // True dynamics for IMU generation
        rtk::Vector3 true_vel_enu(-150.0 * M_PI / 180.0 * std::sin(angle), 150.0 * M_PI / 180.0 * std::cos(angle), 0.0);
        rtk::Vector3 true_acc_enu(-1500.0 * (M_PI/180.0)*(M_PI/180.0) * std::cos(angle), -1500.0 * (M_PI/180.0)*(M_PI/180.0) * std::sin(angle), 0.0);
        rtk::Vector3 true_att(0.0, 0.0, angle);

        // 1. Observations
        rtk::EpochObs b_obs, r_obs;
        rtk::MockGenerator::generateMockObservations(base_ecef, true_ecef, t, b_obs, r_obs);

        // 2. IMU Generation
        rtk::ImuMeas imu;
        rtk::MockGenerator::generateMockImu(true_ecef, true_vel_enu, true_acc_enu, true_att, t, imu);

        // 3. EKF Predict & Update
        ekf.predict(t, &imu);
        ekf.update(b_obs, r_obs, sat_positions);
        
        rtk::Vector3 ekf_ecef(ekf.getState()(rtk::EkfFilter::IDX_POS), 
                              ekf.getState()(rtk::EkfFilter::IDX_POS+1), 
                              ekf.getState()(rtk::EkfFilter::IDX_POS+2));
        rtk::Vector3 cur_enu = rtk::Geodesy::ecefToEnu(ekf_ecef, base_ecef);
        
        rtk::Vector3 att_deg(ekf.getState()(rtk::EkfFilter::IDX_ATT) * 180.0 / M_PI,
                             ekf.getState()(rtk::EkfFilter::IDX_ATT+1) * 180.0 / M_PI,
                             ekf.getState()(rtk::EkfFilter::IDX_ATT+2) * 180.0 / M_PI);
        
        double err = (cur_enu - true_enu).norm();

        // 4. Render Telemetry
        db.render(t, cur_enu, err, r_obs.sat_obs, "INS/GNSS FUSION", att_deg);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        // Maintain 10Hz cadence
        if (duration.count() < 100) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100 - duration.count()));
        }
    }

    std::cout << "\n[GEMINI-RTK] Production demo completed successfully.\n";
    return 0;
}
