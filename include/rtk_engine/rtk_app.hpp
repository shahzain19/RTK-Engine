/**
 * @file rtk_app.hpp
 * @brief Main application orchestrator for the RTK engine.
 */

#ifndef RTK_ENGINE_APP_HPP
#define RTK_ENGINE_APP_HPP

#include "rtk_engine/common.hpp"
#include "rtk_engine/ntrip_client.hpp"
#include "rtk_engine/rtcm3.hpp"
#include "rtk_engine/solver/ekf_filter.hpp"
#include "rtk_engine/ui/dashboard.hpp"
#include "rtk_engine/config_manager.hpp"
#include <memory>
#include <atomic>

namespace rtk {

/**
 * @brief Orchestrates data ingestion, processing, and visualization.
 */
class RtkEngineApp {
public:
    RtkEngineApp();
    ~RtkEngineApp();

    /**
     * @brief Initialize and start the main processing loop.
     * @param config_path Path to the TOML configuration file.
     * @return true if started successfully.
     */
    bool start(const std::string& config_path);

    /** @brief Signals the app to stop. */
    void stop();

private:
    void run();
    void processBaseData();
    void processRoverData(); // Could be from file, serial, or mock

    std::atomic<bool> running_;
    std::unique_ptr<NtripClient> ntrip_client_;
    Rtcm3Parser base_parser_;
    Rtcm3Parser rover_parser_;
    EkfFilter ekf_;
    Dashboard dashboard_;
    
    EpochObs last_base_obs_;
    EpochObs last_rover_obs_;
    std::vector<Vector3> sat_positions_;
    
    double current_gps_time_ = 0.0;
};

} // namespace rtk

#endif // RTK_ENGINE_APP_HPP
