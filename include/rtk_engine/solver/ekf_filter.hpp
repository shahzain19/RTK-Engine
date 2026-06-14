/**
 * @file ekf_filter.hpp
 * @brief Extended Kalman Filter for continuous RTK state estimation.
 */

#ifndef RTK_ENGINE_EKF_FILTER_HPP
#define RTK_ENGINE_EKF_FILTER_HPP

#include "rtk_engine/common.hpp"
#include "rtk_engine/geodesy.hpp"
#include "rtk_engine/solver/solver_utils.hpp"
#include "rtk_engine/solver/signal_processor.hpp"
#include <map>
#include <vector>
#include <set>
#include <iostream>
#include <Eigen/Dense>

namespace rtk {

/**
 * @brief Extended Kalman Filter (EKF) to track rover position, velocity, and ambiguities.
 */
class EkfFilter {
public:
    // --- State Vector Indices ---
    static constexpr int IDX_POS = 0;   ///< ECEF X, Y, Z (0, 1, 2)
    static constexpr int IDX_VEL = 3;   ///< ECEF Vx, Vy, Vz (3, 4, 5)
    static constexpr int IDX_ACC = 6;   ///< ECEF Ax, Ay, Az (6, 7, 8)
    static constexpr int IDX_IFB = 9;   ///< GLONASS Inter-Frequency Bias (9)
    
    // --- Phase 5.1: Inertial Readiness (Bias States) ---
    static constexpr int IDX_BG  = 10;  ///< Gyro Biases (10, 11, 12)
    static constexpr int IDX_BA  = 13;  ///< Accel Biases (13, 14, 15)
    static constexpr int IDX_ATT = 16;  ///< Euler Angles: Roll, Pitch, Yaw (16, 17, 18)
    
    static constexpr int BASE_STATE_SIZE = 19;

    EkfFilter() : initialized_(false), last_time_(0.0) {
        state_ = Eigen::VectorXd::Zero(BASE_STATE_SIZE);
        P_ = Eigen::MatrixXd::Identity(BASE_STATE_SIZE, BASE_STATE_SIZE) * 1e6;
    }

    void initialize(const Vector3& pos, double gps_time) {
        state_ = Eigen::VectorXd::Zero(BASE_STATE_SIZE);
        state_.segment<3>(IDX_POS) << pos.x, pos.y, pos.z;
        
        P_ = Eigen::MatrixXd::Zero(BASE_STATE_SIZE, BASE_STATE_SIZE);
        P_.diagonal().segment<3>(IDX_POS).fill(100.0);   
        P_.diagonal().segment<3>(IDX_VEL).fill(10.0);    
        P_.diagonal().segment<3>(IDX_ACC).fill(1.0);     
        P_(IDX_IFB, IDX_IFB) = 1.0;
        P_.diagonal().segment<3>(IDX_BG).fill(1e-4);     
        P_.diagonal().segment<3>(IDX_BA).fill(1e-2);
        P_.diagonal().segment<3>(IDX_ATT).fill(0.1);     
        
        last_time_ = gps_time;
        initialized_ = true;
        svid_to_idx_.clear();
        prev_gf_obs_.clear();
    }

    /**
     * @brief Propagates the state forward in time.
     * @param current_time Target GPS time.
     * @param imu Optional IMU measurement for inertial-driven prediction.
     */
    void predict(double current_time, const ImuMeas* imu = nullptr) {
        if (!initialized_) return;
        double dt = current_time - last_time_;
        if (dt < 1e-6) return;

        int n = state_.size();
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n, n);
        
        if (imu) {
            // --- INS Prediction Path ---
            const Eigen::Vector3d gyro = Eigen::Vector3d(imu->gyro.x, imu->gyro.y, imu->gyro.z);
            const Eigen::Vector3d acc_body = Eigen::Vector3d(imu->acc.x, imu->acc.y, imu->acc.z);
            
            const Eigen::Vector3d bg = state_.segment<3>(IDX_BG);
            const Eigen::Vector3d ba = state_.segment<3>(IDX_BA);
            const Eigen::Vector3d att = state_.segment<3>(IDX_ATT);
            const Eigen::Vector3d pos = state_.segment<3>(IDX_POS);
            const Eigen::Vector3d vel = state_.segment<3>(IDX_VEL);

            // 1. Update Attitude (Body -> Navigation/ENU)
            Eigen::Vector3d gyro_corr = gyro - bg;
            state_.segment<3>(IDX_ATT) += gyro_corr * dt;

            // 2. Rotate Acceleration to ECEF
            // For simplicity in this demo, we use a basic ENU-based rotation
            // and then transform to ECEF using Geodesy
            Eigen::Matrix3d R_b2n = eulerToRotationMatrix(state_(IDX_ATT), state_(IDX_ATT+1), state_(IDX_ATT+2));
            
            // Get ENU to ECEF rotation at current position
            Eigen::Matrix3d R_n2e = getEnuToEcefMatrix(pos);
            Eigen::Matrix3d R_b2e = R_n2e * R_b2n;

            Eigen::Vector3d acc_corr = acc_body - ba;
            Eigen::Vector3d acc_ecef = R_b2e * acc_corr;

            // 3. Subtract Gravity in ECEF
            Eigen::Vector3d gravity_ecef = -pos.normalized() * GRAVITY();
            Eigen::Vector3d acc_net = acc_ecef + gravity_ecef;

            // 4. Update Position and Velocity
            state_.segment<3>(IDX_POS) += vel * dt + 0.5 * acc_net * dt * dt;
            state_.segment<3>(IDX_VEL) += acc_net * dt;
            state_.segment<3>(IDX_ACC) = acc_net; // Store net acceleration

            // 5. Build Transition Matrix F for Covariance Propagation
            F.block<3, 3>(IDX_POS, IDX_VEL) = Eigen::Matrix3d::Identity() * dt;
            F.block<3, 3>(IDX_VEL, IDX_ATT) = -R_b2e * skewSymmetric(acc_corr) * dt;
        } else {
            // --- Constant Acceleration Fallback ---
            F.block<3, 3>(IDX_POS, IDX_VEL) = Eigen::Matrix3d::Identity() * dt;
            F.block<3, 3>(IDX_POS, IDX_ACC) = Eigen::Matrix3d::Identity() * (0.5 * dt * dt);
            F.block<3, 3>(IDX_VEL, IDX_ACC) = Eigen::Matrix3d::Identity() * dt;
            state_ = F * state_;
        }

        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
        double q_acc = 15.0; 
        Q.block<3, 3>(IDX_ACC, IDX_ACC) = Eigen::Matrix3d::Identity() * q_acc * dt;
        Q.diagonal().segment<3>(IDX_BG).fill(1e-8 * dt);
        Q.diagonal().segment<3>(IDX_BA).fill(1e-6 * dt);
        Q.diagonal().segment<3>(IDX_ATT).fill(1e-5 * dt);
        if (n > BASE_STATE_SIZE) Q.diagonal().tail(n - BASE_STATE_SIZE).fill(1e-6 * dt);

        P_ = F * P_ * F.transpose() + Q;
        last_time_ = current_time;
    }

    void update(const EpochObs& base, const EpochObs& rover, const std::vector<Vector3>& sat_positions) {
        if (!initialized_) return;

        auto ref_base = SolverUtils::selectReferenceSatellites(base);
        auto ref_rover = SolverUtils::selectReferenceSatellites(rover);

        std::vector<int> current_svids;
        for (const auto& sat : base.sat_obs) {
            for (const auto& rsat : rover.sat_obs) {
                if (sat.svid == rsat.svid && sat.sys == rsat.sys) {
                    current_svids.push_back(sat.svid); break;
                }
            }
        }

        manageAmbiguities(base, rover, current_svids, ref_base, ref_rover, sat_positions);

        std::vector<double> residuals;
        std::vector<Eigen::VectorXd> H_rows;
        const Eigen::Vector3d pos_est = state_.segment<3>(IDX_POS);
        const Eigen::Vector3d base_pos(base.ref_pos.x, base.ref_pos.y, base.ref_pos.z);

        for (const auto& [sys, ref_data] : ref_base) {
            if (ref_rover.count(sys) == 0) continue;
            int b_ref_idx = ref_data.sat_idx, r_ref_idx = ref_rover[sys].sat_idx;
            
            if (b_ref_idx >= (int)sat_positions.size()) {
                std::cerr << "[EKF] Error: b_ref_idx out of bounds" << std::endl;
                continue;
            }

            const Eigen::Vector3d s_ref(sat_positions[b_ref_idx].x, sat_positions[b_ref_idx].y, sat_positions[b_ref_idx].z);
            
            double rho_r_ref = (s_ref - pos_est).norm();
            const Eigen::Vector3d e_r_ref = (s_ref - pos_est) / rho_r_ref;
            double SD_P_ref = rover.sat_obs[r_ref_idx].pseudorange - base.sat_obs[b_ref_idx].pseudorange;
            double SD_L_ref = (rover.sat_obs[r_ref_idx].carrier_phase - base.sat_obs[b_ref_idx].carrier_phase) * GPS_L1_WAVELENGTH();

            for (size_t i = 0; i < base.sat_obs.size(); ++i) {
                int svid = base.sat_obs[i].svid;
                if (base.sat_obs[i].sys != sys || static_cast<int>(i) == b_ref_idx || svid_to_idx_.count(svid) == 0) continue;
                
                if (i >= sat_positions.size()) {
                    std::cerr << "[EKF] Error: sat index out of bounds" << std::endl;
                    continue;
                }

                int r_idx = -1;
                for (size_t j = 0; j < rover.sat_obs.size(); ++j) if (rover.sat_obs[j].svid == svid && rover.sat_obs[j].sys == sys) { r_idx = j; break; }
                if (r_idx == -1) continue;

                const Eigen::Vector3d s_i(sat_positions[i].x, sat_positions[i].y, sat_positions[i].z);
                double rho_r_i = (s_i - pos_est).norm();
                const Eigen::Vector3d e_r_i = (s_i - pos_est) / rho_r_i;

                double SD_P_i = rover.sat_obs[r_idx].pseudorange - base.sat_obs[i].pseudorange;
                double SD_L_i = (rover.sat_obs[r_idx].carrier_phase - base.sat_obs[i].carrier_phase) * GPS_L1_WAVELENGTH();

                double DD_P_obs = SD_P_i - SD_P_ref;
                double DD_L_obs = SD_L_i - SD_L_ref;
                double DD_rho_est = (rho_r_i - (s_i - base_pos).norm()) - (rho_r_ref - (s_ref - base_pos).norm());

                int amb_idx = svid_to_idx_[svid];
                double DD_N_est = state_(amb_idx) * GPS_L1_WAVELENGTH();

                Eigen::VectorXd h_p = Eigen::VectorXd::Zero(state_.size());
                h_p.segment<3>(IDX_POS) = e_r_ref - e_r_i;
                if (sys == Constellation::GLONASS) h_p(IDX_IFB) = 1.0;
                H_rows.push_back(h_p);
                residuals.push_back(DD_P_obs - DD_rho_est);

                Eigen::VectorXd h_l = Eigen::VectorXd::Zero(state_.size());
                h_l.segment<3>(IDX_POS) = e_r_ref - e_r_i;
                h_l(amb_idx) = GPS_L1_WAVELENGTH();
                if (sys == Constellation::GLONASS) h_l(IDX_IFB) = 1.0;
                H_rows.push_back(h_l);
                residuals.push_back(DD_L_obs - (DD_rho_est + DD_N_est));
            }
        }

        if (H_rows.empty()) return;
        Eigen::MatrixXd H(H_rows.size(), state_.size());
        Eigen::VectorXd v(residuals.size());
        for (size_t i = 0; i < H_rows.size(); ++i) { H.row(i) = H_rows[i]; v(i) = residuals[i]; }
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(v.size(), v.size());
        for (int i = 0; i < v.size(); i += 2) { R(i, i) = 4.0; R(i+1, i+1) = 0.04; } 

        Eigen::MatrixXd S = H * P_ * H.transpose() + R;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        state_ += K * v;
        P_ = (Eigen::MatrixXd::Identity(state_.size(), state_.size()) - K * H) * P_;
    }

    void manageAmbiguities(const EpochObs& base, const EpochObs& rover, const std::vector<int>& active_svids,
                           std::map<Constellation, ReferenceSat>& ref_b, std::map<Constellation, ReferenceSat>& ref_r,
                           const std::vector<Vector3>& sat_positions) {
        std::set<int> current_set(active_svids.begin(), active_svids.end());
        auto it = svid_to_idx_.begin();
        while (it != svid_to_idx_.end()) {
            int svid = it->first;
            bool lost = (current_set.count(svid) == 0);
            bool slip = false;
            for (const auto& obs : rover.sat_obs) {
                if (obs.svid == svid && obs.signals.size() >= 2) {
                    double lam1 = SPEED_OF_LIGHT() / obs.signals[0].frequency, lam2 = SPEED_OF_LIGHT() / obs.signals[1].frequency;
                    double gf = obs.signals[0].carrier_phase * lam1 - obs.signals[1].carrier_phase * lam2;
                    if (prev_gf_obs_.count(svid) && SignalProcessor::detectCycleSlipGf(gf, prev_gf_obs_[svid])) slip = true;
                    prev_gf_obs_[svid] = gf; break;
                }
            }
            if (lost || slip) {
                int idx = it->second;
                if (slip) std::cout << "[EKF] Cycle slip detected on SVID " << svid << ". Resetting ambiguity.\n";
                removeState(idx);
                int removed_idx = idx; it = svid_to_idx_.erase(it);
                for (auto& pair : svid_to_idx_) if (pair.second > removed_idx) pair.second--;
            } else ++it;
        }

        const Eigen::Vector3d pos_est = state_.segment<3>(IDX_POS);
        const Eigen::Vector3d base_pos(base.ref_pos.x, base.ref_pos.y, base.ref_pos.z);
        for (int svid : active_svids) {
            if (svid_to_idx_.count(svid) == 0) {
                Constellation sys = Constellation::UNKNOWN;
                int b_idx = -1, r_idx = -1;
                for(size_t i=0; i<base.sat_obs.size(); ++i) if(base.sat_obs[i].svid == svid) { b_idx = i; sys = base.sat_obs[i].sys; break; }
                for(size_t i=0; i<rover.sat_obs.size(); ++i) if(rover.sat_obs[i].svid == svid && rover.sat_obs[i].sys == sys) { r_idx = i; break; }
                if (b_idx != -1 && r_idx != -1 && ref_b.count(sys) && ref_r.count(sys)) {
                    int b_ref_idx = ref_b[sys].sat_idx, r_ref_idx = ref_r[sys].sat_idx;
                    const Eigen::Vector3d s_i(sat_positions[b_idx].x, sat_positions[b_idx].y, sat_positions[b_idx].z);
                    const Eigen::Vector3d s_ref(sat_positions[b_ref_idx].x, sat_positions[b_ref_idx].y, sat_positions[b_ref_idx].z);
                    double DD_rho = ((s_i - pos_est).norm() - (s_i - base_pos).norm()) - ((s_ref - pos_est).norm() - (s_ref - base_pos).norm());
                    double DD_L = (rover.sat_obs[r_idx].carrier_phase - base.sat_obs[b_idx].carrier_phase - (rover.sat_obs[r_ref_idx].carrier_phase - base.sat_obs[b_ref_idx].carrier_phase)) * GPS_L1_WAVELENGTH();
                    addState((DD_L - DD_rho) / GPS_L1_WAVELENGTH(), 1000.0);
                    svid_to_idx_[svid] = state_.size() - 1;
                }
            }
        }
    }

    const Eigen::VectorXd& getState() const { return state_; }
    bool isInitialized() const { return initialized_; }

private:
    bool initialized_;
    double last_time_;
    Eigen::VectorXd state_;
    Eigen::MatrixXd P_;
    std::map<int, int> svid_to_idx_;
    std::map<int, double> prev_gf_obs_;

    /** @brief Helper to convert Euler angles (R, P, Y in rad) to 3x3 rotation matrix. */
    Eigen::Matrix3d eulerToRotationMatrix(double r, double p, double y) {
        Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());
        return (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
    }

    /** @brief Helper to get ENU to ECEF rotation matrix at a given ECEF position. */
    Eigen::Matrix3d getEnuToEcefMatrix(const Eigen::Vector3d& pos_ecef) {
        double lat, lon, h;
        Geodesy::ecefToGeodetic(Vector3(pos_ecef.x(), pos_ecef.y(), pos_ecef.z()), lat, lon, h);
        
        double sL = std::sin(lat), cL = std::cos(lat);
        double sG = std::sin(lon), cG = std::cos(lon);

        Eigen::Matrix3d R;
        R << -sG, -sL*cG, cL*cG,
              cG, -sL*sG, cL*sG,
              0.0,   cL,    sL;
        return R;
    }

    /** @brief Helper for skew-symmetric matrix. */
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
        Eigen::Matrix3d m;
        m <<  0,  -v.z(),  v.y(),
            v.z(),   0,   -v.x(),
           -v.y(),  v.x(),   0;
        return m;
    }

    void addState(double val, double var) {
        int n = state_.size();
        state_.conservativeResize(n + 1); state_(n) = val;
        P_.conservativeResize(n + 1, n + 1); P_.row(n).setZero(); P_.col(n).setZero(); P_(n, n) = var;
    }

    void removeState(int idx) {
        int n = state_.size(); if (idx >= n) return;
        int num_move = n - 1 - idx;
        if (num_move > 0) {
            state_.segment(idx, num_move) = state_.segment(idx + 1, num_move);
            P_.block(idx, 0, num_move, n) = P_.block(idx + 1, 0, num_move, n);
            P_.block(0, idx, n - 1, num_move) = P_.block(0, idx + 1, n - 1, num_move);
        }
        state_.conservativeResize(n - 1); P_.conservativeResize(n - 1, n - 1);
    }
};

} // namespace rtk

#endif // RTK_ENGINE_EKF_FILTER_HPP
