/**
 * @file signal_processor.hpp
 * @brief Multi-frequency GNSS signal processing utilities.
 */

#ifndef RTK_ENGINE_SIGNAL_PROCESSOR_HPP
#define RTK_ENGINE_SIGNAL_PROCESSOR_HPP

#include "rtk_engine/common.hpp"
#include <vector>

namespace rtk {

/**
 * @brief Provides algorithms for signal combinations and filtering.
 */
class SignalProcessor {
public:
    /**
     * @brief Ionosphere-Free (IF) Combination.
     * @details Eliminates 1st-order ionospheric delay using two frequencies.
     * @formula P_if = (f1^2 * P1 - f2^2 * P2) / (f1^2 - f2^2)
     */
    static double calculateIonosphereFree(double p1, double f1, double p2, double f2) {
        double f1_sq = f1 * f1;
        double f2_sq = f2 * f2;
        return (f1_sq * p1 - f2_sq * p2) / (f1_sq - f2_sq);
    }

    /**
     * @brief Wide-Lane (WL) Combination.
     * @details Creates a virtual signal with a long wavelength (~86cm for L1/L2).
     * @param phi1 L1 carrier phase (cycles).
     * @param f1 L1 frequency (Hz).
     * @param phi2 L2 carrier phase (cycles).
     * @param f2 L2 frequency (Hz).
     * @param phi_wl Output wide-lane phase.
     * @param lam_wl Output wide-lane wavelength.
     */
    static void calculateWideLane(double phi1, double f1, double phi2, double f2, double& phi_wl, double& lam_wl) {
        phi_wl = phi1 - phi2;
        lam_wl = SPEED_OF_LIGHT / (f1 - f2);
    }

    /**
     * @brief Geometry-Free (GF) Combination.
     * @details Isolates ionospheric delay and cycle slips.
     * @formula P_gf = P1 - P2
     */
    static double calculateGeometryFree(double p1, double p2) {
        return p1 - p2;
    }
};

} // namespace rtk

#endif // RTK_ENGINE_SIGNAL_PROCESSOR_HPP
