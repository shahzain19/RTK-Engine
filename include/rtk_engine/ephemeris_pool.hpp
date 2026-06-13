/**
 * @file ephemeris_pool.hpp
 * @brief Thread-safe storage for satellite broadcast navigation data.
 */

#ifndef RTK_ENGINE_EPHEMERIS_POOL_HPP
#define RTK_ENGINE_EPHEMERIS_POOL_HPP

#include "orbit.hpp"
#include <map>
#include <mutex>

namespace rtk {

/**
 * @brief Singleton pool managing the latest ephemeris for all satellites.
 */
class EphemerisPool {
public:
    static EphemerisPool& getInstance() {
        static EphemerisPool instance;
        return instance;
    }

    /**
     * @brief Update or insert an ephemeris for a satellite.
     * @param eph New ephemeris data.
     */
    void update(const GpsEphemeris& eph) {
        std::lock_guard<std::mutex> lock(mutex_);
        // Update if newer (Toe) or first time
        if (pool_.find(eph.svid) == pool_.end() || eph.toe >= pool_[eph.svid].toe) {
            pool_[eph.svid] = eph;
        }
    }

    /**
     * @brief Retrieve the latest valid ephemeris for a satellite.
     * @param svid Satellite ID.
     * @param t Target GPS time.
     * @param out Reference to store the result.
     * @return true If a valid ephemeris was found.
     */
    bool get(int svid, double t, GpsEphemeris& out) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = pool_.find(svid);
        if (it == pool_.end()) return false;
        
        // In real systems, we check if the ephemeris is too old (> 2 hours)
        if (std::abs(t - it->second.toe) > 7200.0) {
            // return false; // Optional strictness
        }
        
        out = it->second;
        return true;
    }

private:
    EphemerisPool() = default;
    std::map<int, GpsEphemeris> pool_;
    std::mutex mutex_;
};

} // namespace rtk

#endif // RTK_ENGINE_EPHEMERIS_POOL_HPP
