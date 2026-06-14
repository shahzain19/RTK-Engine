#pragma once

#include <string>
#include <vector>
#include "rtk_engine/common.hpp"

namespace rtk {

struct SatelliteOrbit {
    int sat_id;
    double t; // GPS time of week
    Vector3 pos; // ECEF position in meters
    double clock_bias; // Clock bias in seconds
};

struct SatelliteClock {
    int sat_id;
    double t;
    double clock_bias;
};

class Sp3Parser {
public:
    static std::vector<SatelliteOrbit> parse(const std::string& filepath);
};

class ClkParser {
public:
    static std::vector<SatelliteClock> parse(const std::string& filepath);
};

} // namespace rtk
