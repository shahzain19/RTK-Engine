#ifndef RTK_ENGINE_RINEX_HPP
#define RTK_ENGINE_RINEX_HPP

#include "rtk_engine/common.hpp"
#include "rtk_engine/orbit.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>

namespace rtk {

struct RinexObs {
    int year, month, day, hour, minute;
    double second;
    int epoch_flag;
    std::map<std::string, std::map<std::string, double>> sat_data; // SatID -> (ObsType -> Value)
};

class RinexParser {
public:
    RinexParser() = default;

    // Parse a RINEX 3.x Navigation file
    bool parseNavFile(const std::string& filename, std::vector<GpsEphemeris>& ephemerides) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;

        std::string line;
        bool in_header = true;

        while (std::getline(file, line)) {
            if (in_header) {
                if (line.find("END OF HEADER") != std::string::npos) in_header = false;
                continue;
            }

            if (line.empty()) continue;

            // GPS Ephemeris record starts with SatID (e.g., G01)
            if (line[0] == 'G') {
                try {
                    GpsEphemeris eph;
                    eph.svid = std::stoi(line.substr(1, 2));
                    
                    // First line: Year Month Day Hour Minute Second, af0, af1, af2
                    int y = std::stoi(line.substr(4, 4));
                    int m = std::stoi(line.substr(9, 2));
                    int d = std::stoi(line.substr(12, 2));
                    int hh = std::stoi(line.substr(15, 2));
                    int mm = std::stoi(line.substr(18, 2));
                    double ss = std::stod(line.substr(21, 2));
                    
                    eph.af0 = std::stod(line.substr(23, 19));
                    eph.af1 = std::stod(line.substr(42, 19));
                    eph.af2 = std::stod(line.substr(61, 19));

                    // Broadcast Orbit - Line 1 (IODE, Crs, Delta n, M0)
                    if (!std::getline(file, line)) break;
                    eph.crs = std::stod(line.substr(23, 19));
                    eph.delta_n = std::stod(line.substr(42, 19));
                    eph.m0 = std::stod(line.substr(61, 19));

                    // Broadcast Orbit - Line 2 (Cuc, e, Cus, sqrt(A))
                    if (!std::getline(file, line)) break;
                    eph.cuc = std::stod(line.substr(4, 19));
                    eph.e = std::stod(line.substr(23, 19));
                    eph.cus = std::stod(line.substr(42, 19));
                    eph.sqrt_a = std::stod(line.substr(61, 19));

                    // Broadcast Orbit - Line 3 (Toe, Cic, OMEGA0, Cis)
                    if (!std::getline(file, line)) break;
                    eph.toe = std::stod(line.substr(4, 19));
                    eph.cic = std::stod(line.substr(23, 19));
                    eph.omg0 = std::stod(line.substr(42, 19));
                    eph.cis = std::stod(line.substr(61, 19));

                    // Broadcast Orbit - Line 4 (i0, Crc, omega, OMEGA DOT)
                    if (!std::getline(file, line)) break;
                    eph.i0 = std::stod(line.substr(4, 19));
                    eph.crc = std::stod(line.substr(23, 19));
                    eph.omega = std::stod(line.substr(42, 19));
                    eph.omg_dot = std::stod(line.substr(61, 19));

                    // Broadcast Orbit - Line 5 (IDOT, Codes, GPS Week, L2 P data flag)
                    if (!std::getline(file, line)) break;
                    eph.idot = std::stod(line.substr(4, 19));

                    // Broadcast Orbit - Line 6 (SV accuracy, Health, TGD, IODC)
                    if (!std::getline(file, line)) break;
                    eph.tgd = std::stod(line.substr(42, 19));

                    // Broadcast Orbit - Line 7 (Transmission time, Fit Interval)
                    if (!std::getline(file, line)) break;
                    
                    eph.t_oc = eph.toe;
                    ephemerides.push_back(eph);
                } catch (...) {
                    continue;
                }
            }
        }
        return true;
    }

    bool parseObsFile(const std::string& filename, std::vector<RinexObs>& observations) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;

        std::string line;
        bool in_header = true;
        std::map<char, std::vector<std::string>> sys_obs_types;

        while (std::getline(file, line)) {
            if (in_header) {
                if (line.find("SYS / # / OBS TYPES") != std::string::npos) {
                    char sys = line[0];
                    int num_types = std::stoi(line.substr(3, 3));
                    std::vector<std::string> types;
                    
                    size_t pos = 7;
                    for (int i = 0; i < num_types; ++i) {
                        if (pos + 3 > 60) {
                            if (!std::getline(file, line)) break;
                            pos = 7;
                        }
                        types.push_back(line.substr(pos, 3));
                        pos += 4;
                    }
                    sys_obs_types[sys] = types;
                }
                if (line.find("END OF HEADER") != std::string::npos) {
                    in_header = false;
                }
                continue;
            }

            if (line.empty()) continue;

            if (line[0] == '>') {
                RinexObs obs;
                try {
                    obs.year = std::stoi(line.substr(2, 4));
                    obs.month = std::stoi(line.substr(7, 2));
                    obs.day = std::stoi(line.substr(10, 2));
                    obs.hour = std::stoi(line.substr(13, 2));
                    obs.minute = std::stoi(line.substr(16, 2));
                    obs.second = std::stod(line.substr(19, 11));
                    obs.epoch_flag = std::stoi(line.substr(31, 2));
                    int num_sats = std::stoi(line.substr(33, 3));

                    for (int i = 0; i < num_sats; ++i) {
                        if (!std::getline(file, line)) break;
                        std::string sat_id = line.substr(0, 3);
                        char sys = sat_id[0];
                        
                        if (sys_obs_types.count(sys)) {
                            const auto& types = sys_obs_types.at(sys);
                            size_t pos = 3;
                            for (const auto& type : types) {
                                if (pos + 14 <= line.length()) {
                                    std::string val_str = line.substr(pos, 14);
                                    try {
                                        double val = std::stod(val_str);
                                        obs.sat_data[sat_id][type] = val;
                                    } catch (...) {}
                                }
                                pos += 16;
                            }
                        }
                    }
                    observations.push_back(std::move(obs));
                } catch (...) {
                    continue;
                }
            }
        }
        return true;
    }
};

} // namespace rtk

#endif // RTK_ENGINE_RINEX_HPP
