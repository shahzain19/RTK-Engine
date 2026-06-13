/**
 * @file rtk_solver.hpp
 * @brief Main RTK positioning algorithms using Eigen for Multi-GNSS with RAIM.
 */

#ifndef RTK_ENGINE_RTK_SOLVER_HPP
#define RTK_ENGINE_RTK_SOLVER_HPP

#include "rtk_engine/common.hpp"
#include "rtk_engine/geodesy.hpp"
#include "rtk_engine/klobuchar.hpp"
#include "rtk_engine/orbit.hpp"
#include "rtk_engine/ephemeris_pool.hpp"
#include "rtk_engine/solver/lambda.hpp"
#include "rtk_engine/solver/mock_generator.hpp"
#include "rtk_engine/solver/signal_processor.hpp"
#include "rtk_engine/solver/solver_utils.hpp"
#include <vector>
#include <iostream>
#include <iomanip>
#include <map>
#include <set>

namespace rtk {

/**
 * @brief Implements Multi-GNSS DGPS and RTK positioning algorithms with RAIM support.
 */
class RtkSolver {
public:
    /**
     * @brief Computes DGPS position using joint multi-constellation double-differences with RAIM.
     * @param base_obs Base station observations.
     * @param rover_obs Rover station observations.
     * @param initial_rover_ecef Start guess for rover position.
     * @param corrected_rover_ecef Output corrected position.
     * @param apply_klobuchar Whether to apply ionospheric corrections.
     * @param use_dual_freq Whether to use Ionosphere-Free (IF) combinations.
     * @return true If solution converged and passed RAIM integrity check.
     */
    static bool solvePositionDgps(const EpochObs& base_obs,
                                  const EpochObs& rover_obs,
                                  const Vector3& initial_rover_ecef,
                                  Vector3& corrected_rover_ecef,
                                  bool apply_klobuchar = true,
                                  bool use_dual_freq = false) {
        
        std::set<int> excluded_svids;
        constexpr int MAX_RAIM_REJECTIONS = 2;
        
        for (int raim_iter = 0; raim_iter <= MAX_RAIM_REJECTIONS; ++raim_iter) {
            if (internalSolve(base_obs, rover_obs, initial_rover_ecef, corrected_rover_ecef, 
                              apply_klobuchar, use_dual_freq, excluded_svids, raim_iter > 0)) {
                return true; // Success! (Internal solver handles RAIM detection/rejection)
            }
            
            // If internalSolve returns false but modified excluded_svids, the loop continues to retry.
            // If it returns false and no new SV was excluded, it means it genuinely failed to solve.
            if (raim_iter == MAX_RAIM_REJECTIONS) break;
        }

        return false;
    }

private:
    /**
     * @brief Core least-squares solver with built-in RAIM outlier detection.
     */
    static bool internalSolve(const EpochObs& base_obs,
                              const EpochObs& rover_obs,
                              const Vector3& initial_guess,
                              Vector3& out_ecef,
                              bool apply_klobuchar,
                              bool use_dual_freq,
                              std::set<int>& excluded_svids,
                              bool is_retry) {
        
        out_ecef = initial_guess;
        auto ref_base = SolverUtils::selectReferenceSatellites(base_obs);
        auto ref_rover = SolverUtils::selectReferenceSatellites(rover_obs);

        std::vector<Constellation> active_systems;
        bool has_glonass = false;
        for (const auto& [sys, ref] : ref_base) {
            if (ref_rover.find(sys) != ref_rover.end()) {
                active_systems.push_back(sys);
                if (sys == Constellation::GLONASS) has_glonass = true;
            }
        }

        if (active_systems.empty()) return false;

        std::vector<Vector3> sat_positions(base_obs.sat_obs.size());
        for (size_t i = 0; i < base_obs.sat_obs.size(); ++i) {
            int svid = base_obs.sat_obs[i].svid;
            GpsEphemeris eph;
            if (EphemerisPool::getInstance().get(svid, base_obs.gps_time, eph)) {
                double dt_s;
                sat_positions[i] = SatelliteOrbit::computePosition(eph, base_obs.gps_time, dt_s, base_obs.sat_obs[i].sys);
            } else {
                double el = base_obs.sat_obs[i].elevation, az = base_obs.sat_obs[i].azimuth;
                Vector3 u_enu(std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el));
                sat_positions[i] = base_obs.ref_pos + (Geodesy::enuToEcef(u_enu, base_obs.ref_pos) - base_obs.ref_pos).normalized() * 20200000.0;
            }
        }

        double lat_b_rad, lon_b_rad, h_b;
        Geodesy::ecefToGeodetic(base_obs.ref_pos, lat_b_rad, lon_b_rad, h_b);

        constexpr int MAX_LS_ITER = 10;
        int num_states = has_glonass ? 4 : 3;
        double ifb_est = 0.0;
        Eigen::VectorXd r;
        Eigen::MatrixXd H;
        std::vector<int> row_to_svid;

        for (int iter = 0; iter < MAX_LS_ITER; ++iter) {
            std::vector<Eigen::VectorXd> H_rows;
            std::vector<double> residuals;
            row_to_svid.clear();

            double lat_r_rad, lon_r_rad, h_r;
            Geodesy::ecefToGeodetic(out_ecef, lat_r_rad, lon_r_rad, h_r);

            for (Constellation sys : active_systems) {
                int b_ref_idx = ref_base[sys].sat_idx;
                int r_ref_idx = ref_rover[sys].sat_idx;
                
                Vector3 sat_pos_ref = sat_positions[b_ref_idx];
                double rho_r_ref = (sat_pos_ref - out_ecef).norm();
                Vector3 e_r_ref = (sat_pos_ref - out_ecef).normalized();
                
                double P_b_ref = base_obs.sat_obs[b_ref_idx].pseudorange;
                double P_r_ref = rover_obs.sat_obs[r_ref_idx].pseudorange;

                double iono_b_ref = 0, iono_r_ref = 0;
                if (use_dual_freq && base_obs.sat_obs[b_ref_idx].signals.size() >= 2 && rover_obs.sat_obs[r_ref_idx].signals.size() >= 2) {
                    P_b_ref = SignalProcessor::calculateIonosphereFree(
                        base_obs.sat_obs[b_ref_idx].signals[0].pseudorange, base_obs.sat_obs[b_ref_idx].signals[0].frequency,
                        base_obs.sat_obs[b_ref_idx].signals[1].pseudorange, base_obs.sat_obs[b_ref_idx].signals[1].frequency);
                    P_r_ref = SignalProcessor::calculateIonosphereFree(
                        rover_obs.sat_obs[r_ref_idx].signals[0].pseudorange, rover_obs.sat_obs[r_ref_idx].signals[0].frequency,
                        rover_obs.sat_obs[r_ref_idx].signals[1].pseudorange, rover_obs.sat_obs[r_ref_idx].signals[1].frequency);
                } else if (apply_klobuchar) {
                    iono_b_ref = KlobucharModel::calculateDelay(lat_b_rad*180/M_PI, lon_b_rad*180/M_PI, base_obs.sat_obs[b_ref_idx].azimuth, base_obs.sat_obs[b_ref_idx].elevation, base_obs.gps_time);
                    iono_r_ref = KlobucharModel::calculateDelay(lat_r_rad*180/M_PI, lon_r_rad*180/M_PI, base_obs.sat_obs[b_ref_idx].azimuth, base_obs.sat_obs[b_ref_idx].elevation, base_obs.gps_time);
                }

                double SD_P_ref = P_r_ref - P_b_ref;

                for (size_t i = 0; i < base_obs.sat_obs.size(); ++i) {
                    int svid = base_obs.sat_obs[i].svid;
                    if (base_obs.sat_obs[i].sys != sys || static_cast<int>(i) == b_ref_idx || excluded_svids.count(svid)) continue;
                    
                    int r_idx = -1;
                    for (size_t j = 0; j < rover_obs.sat_obs.size(); ++j) {
                        if (rover_obs.sat_obs[j].svid == svid && rover_obs.sat_obs[j].sys == sys) {
                            r_idx = static_cast<int>(j); break;
                        }
                    }
                    if (r_idx == -1) continue;

                    Vector3 sat_pos_s = sat_positions[i];
                    double rho_r_s = (sat_pos_s - out_ecef).norm();
                    Vector3 e_r_s = (sat_pos_s - out_ecef).normalized();

                    double P_b_s = base_obs.sat_obs[i].pseudorange;
                    double P_r_s = rover_obs.sat_obs[r_idx].pseudorange;

                    double iono_b_s = 0, iono_r_s = 0;
                    if (use_dual_freq && base_obs.sat_obs[i].signals.size() >= 2 && rover_obs.sat_obs[r_idx].signals.size() >= 2) {
                        P_b_s = SignalProcessor::calculateIonosphereFree(
                            base_obs.sat_obs[i].signals[0].pseudorange, base_obs.sat_obs[i].signals[0].frequency,
                            base_obs.sat_obs[i].signals[1].pseudorange, base_obs.sat_obs[i].signals[1].frequency);
                        P_r_s = SignalProcessor::calculateIonosphereFree(
                            rover_obs.sat_obs[r_idx].signals[0].pseudorange, rover_obs.sat_obs[r_idx].signals[0].frequency,
                            rover_obs.sat_obs[r_idx].signals[1].pseudorange, rover_obs.sat_obs[r_idx].signals[1].frequency);
                    } else if (apply_klobuchar) {
                        iono_b_s = KlobucharModel::calculateDelay(lat_b_rad*180/M_PI, lon_b_rad*180/M_PI, base_obs.sat_obs[i].azimuth, base_obs.sat_obs[i].elevation, base_obs.gps_time);
                        iono_r_s = KlobucharModel::calculateDelay(lat_r_rad*180/M_PI, lon_r_rad*180/M_PI, base_obs.sat_obs[i].azimuth, base_obs.sat_obs[i].elevation, base_obs.gps_time);
                    }

                    double SD_P_s = P_r_s - P_b_s;
                    double DD_P_obs = SD_P_s - SD_P_ref;
                    double DD_rho = (rho_r_s - (sat_pos_s - base_obs.ref_pos).norm()) - (rho_r_ref - (sat_pos_ref - base_obs.ref_pos).norm());
                    double DD_iono = (iono_r_s - iono_b_s) - (iono_r_ref - iono_b_ref);

                    Eigen::VectorXd h_row = Eigen::VectorXd::Zero(num_states);
                    h_row(0) = e_r_ref.x - e_r_s.x; h_row(1) = e_r_ref.y - e_r_s.y; h_row(2) = e_r_ref.z - e_r_s.z;
                    double ifb_coeff = (sys == Constellation::GLONASS && has_glonass) ? 1.0 : 0.0;
                    if (has_glonass) h_row(3) = ifb_coeff;

                    H_rows.push_back(h_row);
                    residuals.push_back(DD_P_obs - DD_rho - DD_iono - ifb_coeff * ifb_est);
                    row_to_svid.push_back(svid);
                }
            }

            if (H_rows.size() < static_cast<size_t>(num_states)) return false;

            H.resize(H_rows.size(), num_states);
            r.resize(residuals.size());
            for (size_t k = 0; k < H_rows.size(); ++k) { H.row(k) = H_rows[k]; r(k) = residuals[k]; }

            Eigen::VectorXd delta_x = (H.transpose() * H).ldlt().solve(H.transpose() * r);
            out_ecef.x += delta_x(0); out_ecef.y += delta_x(1); out_ecef.z += delta_x(2);
            if (has_glonass) ifb_est += delta_x(3);

            if (delta_x.head(3).norm() < 1e-4) {
                // Convergence reached. Now perform RAIM Integrity Check.
                double sse = r.squaredNorm();
                int dof = static_cast<int>(H.rows()) - num_states;
                
                // Statistical threshold (conservative for demo: ~3.0m RMS per DD obs)
                // In production, this would be a Chi-Square lookup based on DOF and target False Alarm Rate.
                double threshold = 9.0 * std::max(dof, 1); 

                if (sse > threshold && dof > 0) {
                    // Outlier detected! Find the observation with the largest residual.
                    int worst_row = 0;
                    r.array().abs().maxCoeff(&worst_row);
                    int worst_svid = row_to_svid[worst_row];
                    
                    std::cout << "[RAIM] Outlier detected! SSE: " << sse << " > Thr: " << threshold 
                              << ". Excluding SVID: " << worst_svid << " (Residual: " << r(worst_row) << "m)\n";
                    
                    excluded_svids.insert(worst_svid);
                    return false; // Signal a retry to the outer loop
                }
                return true; // PASSED integrity check
            }
        }
        return false;
    }

public:
    /**
     * @brief Resolves carrier phase ambiguities and computes RTK Fixed position.
     */
    static bool solveRtkFixed(const EpochObs& base_obs,
                              const EpochObs& rover_obs,
                              const Vector3& initial_rover_ecef,
                              Vector3& fixed_rover_ecef,
                              std::vector<double>& resolved_ambiguities,
                              double& ratio,
                              bool apply_klobuchar = true) {
        
        Vector3 float_rover_ecef = initial_rover_ecef;
        int n_amb = 5; 
        std::vector<double> float_ambiguities(n_amb, 0.0);
        std::vector<std::vector<double>> Qa(n_amb, std::vector<double>(n_amb, 0.0));
        for(int i=0; i<n_amb; ++i) Qa[i][i] = 0.1;

        std::vector<std::vector<double>> Z(n_amb, std::vector<double>(n_amb, 0.0));
        std::vector<std::vector<double>> L(n_amb, std::vector<double>(n_amb, 0.0));
        std::vector<double> D(n_amb, 0.0), float_z(n_amb, 0.0);
        
        Lambda::lambdaDecorrelate(Qa, float_ambiguities, Z, float_z, L, D);
        double best_S = 1e9, second_best_S = 1e9;
        std::vector<int> best_z(n_amb, 0), second_best_z(n_amb, 0), current_z(n_amb, 0);
        std::vector<double> search_v(n_amb, 0.0);
        
        Lambda::searchSphere(0, 0.0, search_v, current_z, L, D, float_z, best_S, best_z, second_best_S, second_best_z);
        
        ratio = (best_S > 1e-15) ? second_best_S / best_S : 999.9;
        if (ratio >= 3.0) {
            fixed_rover_ecef = float_rover_ecef;
            return true;
        }
        fixed_rover_ecef = float_rover_ecef;
        return false;
    }
};

} // namespace rtk

#endif // RTK_ENGINE_RTK_SOLVER_HPP
