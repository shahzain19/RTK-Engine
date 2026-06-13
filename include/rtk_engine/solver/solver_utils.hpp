/**
 * @file solver_utils.hpp
 * @brief Helper utilities for RTK solver logic.
 */

#ifndef RTK_ENGINE_SOLVER_UTILS_HPP
#define RTK_ENGINE_SOLVER_UTILS_HPP

#include "rtk_engine/types.hpp"
#include <vector>
#include <map>

namespace rtk {

/** @brief Selected reference satellite for double-differencing. */
struct ReferenceSat {
    int sat_idx; ///< Index in satellite observation list.
    int svid;    ///< Satellite ID.
};

/**
 * @brief Utility class for solver pre-processing.
 */
class SolverUtils {
public:
    /**
     * @brief Selects one high-elevation reference satellite per constellation.
     * @param obs Set of satellite observations.
     * @return std::map Mapping constellations to their reference satellites.
     */
    static std::map<Constellation, ReferenceSat> selectReferenceSatellites(const EpochObs& obs) {
        std::map<Constellation, ReferenceSat> ref_map;
        std::map<Constellation, double> max_el;

        for (size_t i = 0; i < obs.sat_obs.size(); ++i) {
            const auto& sat = obs.sat_obs[i];
            if (max_el.find(sat.sys) == max_el.end() || sat.elevation > max_el[sat.sys]) {
                max_el[sat.sys] = sat.elevation;
                ref_map[sat.sys] = {static_cast<int>(i), sat.svid};
            }
        }
        return ref_map;
    }
};

} // namespace rtk

#endif // RTK_ENGINE_SOLVER_UTILS_HPP
