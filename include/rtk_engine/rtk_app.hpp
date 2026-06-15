/**
 * @file rtk_app.hpp
 * @brief Main application orchestrator for the RTK engine.
 */

#ifndef RTK_ENGINE_APP_HPP
#define RTK_ENGINE_APP_HPP

#include "rtk_engine/common.hpp"
#include "rtk_engine/ntrip_client.hpp"
#include "rtk_engine/io/serial_reader.hpp"
#include "rtk_engine/output/nmea_formatter.hpp"
#include "rtk_engine/output/rinex_writer.hpp"
#include "rtk_engine/rtcm3.hpp"
#include "rtk_engine/rinex.hpp"
#include "rtk_engine/solver/ekf_filter.hpp"
#include "rtk_engine/ui/dashboard.hpp"
#include "rtk_engine/config_manager.hpp"
#include "rtk_engine/rtk_api.h"

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>

namespace rtk {

/**
 * @brief Orchestrates data ingestion, processing, and visualization.
 */
class RtkEngineApp {
public:
    RtkEngineApp();
    ~RtkEngineApp();

    /**
     * @brief Initialize and start the processing engine.
     * @param config_path Path to the TOML configuration file.
     * @param background If true, runs the processing loop in a separate thread.
     * @return true if started successfully.
     */
    bool start(const std::string& config_path, bool background = false);

    /** @brief Signals the app to stop. */
    void stop();

    /** @brief Returns true if the engine is currently running. */
    bool isRunning() const { return running_; }

    /** @brief Feed raw data into the rover parser. */
    void feedRoverData(const uint8_t* data, size_t len);

    /** @brief Inject IMU measurement. */
    void injectImu(const rtk_imu_t& imu);

    /** @brief Retrieve the latest computed solution. */
    bool getLatestSolution(rtk_solution_t& sol);

private:
    void run();
    void processBaseData();
    void processRoverData();

    std::atomic<bool> running_;
    std::unique_ptr<std::thread> worker_thread_;
    mutable std::mutex data_mutex_;
    
    std::unique_ptr<NtripClient> ntrip_client_;
    std::unique_ptr<SerialReader> serial_reader_;
    std::unique_ptr<RinexWriter> rinex_writer_;
    Rtcm3Parser base_parser_;
    Rtcm3Parser rover_parser_;
    RinexParser rinex_parser_;
    std::vector<RinexObs> rover_obs_file_;
    size_t rover_obs_idx_ = 0;
    EkfFilter ekf_;
    Dashboard dashboard_;
    
    EpochObs last_base_obs_;
    EpochObs last_rover_obs_;
    std::vector<Vector3> sat_positions_;
    
    double current_gps_time_ = 0.0;
    rtk_solution_t latest_sol_{};
};

} // namespace rtk

#endif // RTK_ENGINE_APP_HPP
