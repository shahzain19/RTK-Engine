/**
 * @file signal_processor.hpp
 * @brief Multi-frequency GNSS signal processing utilities.
 */

#ifndef RTK_ENGINE_SIGNAL_PROCESSOR_HPP
#define RTK_ENGINE_SIGNAL_PROCESSOR_HPP

#include "rtk_engine/common.hpp"
#include <vector>
#include <cmath>

namespace rtk {

/**
 * @brief Provides algorithms for signal combinations and filtering.
 */
class SignalProcessor {
public:
    /**
     * @brief Ionosphere-Free (IF) Combination.
     */
    static double calculateIonosphereFree(double p1, double f1, double p2, double f2) {
        double f1_sq = f1 * f1;
        double f2_sq = f2 * f2;
        return (f1_sq * p1 - f2_sq * p2) / (f1_sq - f2_sq);
    }

    /**
     * @brief Wide-Lane (WL) Combination.
     */
    static void calculateWideLane(double phi1, double f1, double phi2, double f2, double& phi_wl, double& lam_wl) {
        phi_wl = phi1 - phi2;
        lam_wl = SPEED_OF_LIGHT / (f1 - f2);
    }

    /**
     * @brief Geometry-Free (GF) Combination.
     */
    static double calculateGeometryFree(double p1, double p2) {
        return p1 - p2;
    }

    /**
     * @brief Detect cycle slips using the Geometry-Free (GF) phase jump method.
     * @param gf_current Current GF combination (L1 - L2 in cycles or meters).
     * @param gf_previous Previous GF combination.
     * @param threshold Jump threshold (e.g. 0.05m).
     * @return true If a cycle slip is detected.
     */
    static bool detectCycleSlipGf(double gf_current, double gf_previous, double threshold = 0.05) {
        return std::abs(gf_current - gf_previous) > threshold;
    }
};

} // namespace rtk

#endif // RTK_ENGINE_SIGNAL_PROCESSOR_HPP
