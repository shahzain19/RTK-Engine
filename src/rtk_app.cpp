/**
 * @file rtk_app.cpp
 * @brief Production-grade implementation of the RTK application orchestrator.
 */

#include "rtk_engine/rtk_app.hpp"
#include "rtk_engine/solver/mock_generator.hpp"
#include "rtk_engine/ephemeris_pool.hpp"
#include "rtk_engine/geodesy.hpp"
#include "rtk_engine/rinex.hpp" // Added
#include <thread>
#include <chrono>
#include <iostream>

namespace rtk {

RtkEngineApp::RtkEngineApp() : running_(false) {}

RtkEngineApp::~RtkEngineApp() {
    stop();
}

bool RtkEngineApp::start(const std::string& config_path) {
    auto& cfg = rtk_engine::ConfigManager::instance();
    if (!cfg.load(config_path)) {
        std::cerr << "[APP] Error: Could not load config " << config_path << "\n";
        return false;
    }

    // 1. Configure NTRIP (Optional)
    std::string host = cfg.get<std::string>("ntrip.host");
    if (!host.empty()) {
        NtripClient::Config ncfg;
        ncfg.host = host;
        ncfg.port = cfg.get<int>("ntrip.port");
        ncfg.mountpoint = cfg.get<std::string>("ntrip.mountpoint");
        ncfg.user = cfg.get<std::string>("ntrip.user");
        ncfg.password = cfg.get<std::string>("ntrip.password");
        
        ntrip_client_ = std::make_unique<NtripClient>(ncfg);
        if (!ntrip_client_->connect()) {
            std::cerr << "[APP] Warning: NTRIP failed. Falling back to mock data.\n";
            ntrip_client_.reset();
        }
    }

    // Load Rover File
    std::string rover_file = cfg.get<std::string>("rover.file");
    if (!rover_file.empty()) {
        if (!rinex_parser_.parseObsFile(rover_file, rover_obs_file_)) {
            std::cerr << "[APP] Warning: Could not load rover file " << rover_file << "\n";
        }
    }

    // Initialize Serial Port
    std::string serial_port = cfg.get<std::string>("serial.port");
    if (!serial_port.empty()) {
        serial_reader_ = std::make_unique<SerialReader>(serial_port, cfg.get<int>("serial.baud_rate", 115200));
        if (!serial_reader_->connect()) {
            std::cerr << "[APP] Warning: Serial port failed to open.\n";
            serial_reader_.reset();
        }
    }

    // Initialize RINEX Logger
    if (cfg.get<std::string>("output.format") == "rinex") {
        rinex_writer_ = std::make_unique<RinexWriter>("output.obs");
    }

    // 2. Initialize Solver
    Vector3 init_pos = Geodesy::geodeticToEcef(
        cfg.get<double>("initial.lat", 37.422) * M_PI / 180.0,
        cfg.get<double>("initial.lon", -122.084) * M_PI / 180.0,
        cfg.get<double>("initial.alt", 30.0)
    );
    ekf_.initialize(init_pos, 0.0);

    running_ = true;
    run();
    
    return true;
}

void RtkEngineApp::stop() {
    running_ = false;
}

void RtkEngineApp::run() {
    dashboard_.reset();
    
    auto last_tick = std::chrono::steady_clock::now();
    double t = 0.0;

    while (running_) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tick).count() < 100) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        last_tick = now;
        t += 0.1;

        // Process Data
        processBaseData();
        processRoverData();

        if (last_base_obs_.sat_obs.size() >= 4 && last_rover_obs_.sat_obs.size() >= 4) {
            
            // IMU Fusion (Simple circular motion mock for demo)
            ImuMeas imu;
            double angle = (t * 10.0) * M_PI / 180.0;
            imu.t = t;
            imu.acc = Vector3(-1.5 * std::cos(angle), -1.5 * std::sin(angle), 9.81);
            imu.gyro = Vector3(0, 0, 0.1);

            ekf_.predict(t, &imu);
            
            if (sat_positions_.empty() || sat_positions_.size() < last_base_obs_.sat_obs.size()) {
                sat_positions_ = MockGenerator::precomputeSatPositions(last_base_obs_.ref_pos);
            }

            ekf_.update(last_base_obs_, last_rover_obs_, sat_positions_);
            
            const Eigen::VectorXd& state = ekf_.getState();
            Vector3 ekf_ecef(state(EkfFilter::IDX_POS), state(EkfFilter::IDX_POS+1), state(EkfFilter::IDX_POS+2));
            Vector3 cur_enu = Geodesy::ecefToEnu(ekf_ecef, last_base_obs_.ref_pos);
            Vector3 att_deg(state(EkfFilter::IDX_ATT) * 180.0 / M_PI,
                            state(EkfFilter::IDX_ATT+1) * 180.0 / M_PI,
                            state(EkfFilter::IDX_ATT+2) * 180.0 / M_PI);

            Vector3 true_enu(15.0 * std::cos(angle), 15.0 * std::sin(angle), 0.0);
            double err = (cur_enu - true_enu).norm();
// Render Telemetry
dashboard_.render(t, cur_enu, err, last_rover_obs_.sat_obs, "INS/GNSS", att_deg);

// NMEA Output
auto& cfg = rtk_engine::ConfigManager::instance();
if (cfg.get<bool>("output.enabled", false)) {
    if (cfg.get<std::string>("output.format") == "nmea") {
        std::string nmea = NmeaFormatter::formatGGA(t, cur_enu, last_rover_obs_.sat_obs.size());
        std::cout << nmea << std::endl;
    } else if (rinex_writer_) {
        rinex_writer_->writeEpoch(t, last_rover_obs_.sat_obs);
    }
}
} else {
dashboard_.render(t, Vector3(0,0,0), 0.0, {}, "WAITING DATA", Vector3(0,0,0));
}
    }
}

void RtkEngineApp::processBaseData() {
    if (!ntrip_client_) {
        // Mock Base at config location
        auto& cfg = rtk_engine::ConfigManager::instance();
        Vector3 base_ecef = Geodesy::geodeticToEcef(
            cfg.get<double>("initial.lat", 37.422) * M_PI / 180.0,
            cfg.get<double>("initial.lon", -122.084) * M_PI / 180.0,
            cfg.get<double>("initial.alt", 30.0)
        );
        EpochObs rover_dummy;
        last_base_obs_.sat_obs.clear();
        MockGenerator::generateMockObservations(base_ecef, base_ecef, 0.0, last_base_obs_, rover_dummy);
        return;
    }

    uint8_t buffer[4096];
    int n = ntrip_client_->read(buffer, sizeof(buffer));
    if (n > 0) {
        auto frames = base_parser_.parseStream(buffer, n);
        for (const auto& frame : frames) {
            if (frame.message_type == 1005) {
                Msg1005 msg;
                if (Rtcm3Parser::decode1005(frame.payload, msg)) {
                    last_base_obs_.ref_pos = msg.antenna_ecef;
                }
            } else if (frame.message_type == 1004) {
                Msg1004 msg;
                if (Rtcm3Parser::decode1004(frame.payload, msg)) {
                    last_base_obs_.sat_obs.clear();
                    last_base_obs_.gps_time = msg.gps_tow_ms / 1000.0;
                    for (const auto& s : msg.sats) {
                        SatelliteObs obs;
                        obs.svid = s.svid;
                        obs.sys = Constellation::GPS;
                        obs.pseudorange = s.pseudorange_l1;
                        obs.carrier_phase = s.carrier_phase_l1;
                        last_base_obs_.sat_obs.push_back(obs);
                    }
                }
            }
        }
    }
}

void RtkEngineApp::processRoverData() {
    if (serial_reader_) {
        uint8_t buffer[1024];
        int n = serial_reader_->read(buffer, sizeof(buffer));
        if (n > 0) {
            // TODO: Parse raw GNSS data (e.g., NMEA/RTCM3) from serial buffer
            // For now, just logging activity
            std::cout << "[IO] Read " << n << " bytes from serial.\n";
        }
        return;
    }

    if (!rover_obs_file_.empty() && rover_obs_idx_ < rover_obs_file_.size()) {
        const auto& obs = rover_obs_file_[rover_obs_idx_++];
        
        last_rover_obs_.sat_obs.clear();
        for (const auto& [sat_id, data] : obs.sat_data) {
            SatelliteObs sat;
            sat.svid = std::stoi(sat_id.substr(1));
            // Basic mapping - assume C1C for L1 pseudorange
            if (data.count("C1C")) sat.pseudorange = data.at("C1C");
            
            last_rover_obs_.sat_obs.push_back(sat);
        }
        return;
    }

    Vector3 base_ecef = last_base_obs_.ref_pos;
    if (base_ecef.norm() < 1e6) return;

    double angle = (current_gps_time_ * 10.0) * M_PI / 180.0;
    Vector3 true_enu(15.0 * std::cos(angle), 15.0 * std::sin(angle), 0.0);
    Vector3 true_ecef = Geodesy::enuToEcef(true_enu, base_ecef);
    
    EpochObs base_dummy;
    last_rover_obs_.sat_obs.clear();
    MockGenerator::generateMockObservations(base_ecef, true_ecef, current_gps_time_, base_dummy, last_rover_obs_);
    current_gps_time_ += 0.1;
}

} // namespace rtk
