#include "rtk_engine/ppp_parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

namespace rtk {

std::vector<SatelliteOrbit> Sp3Parser::parse(const std::string& filepath) {
    std::vector<SatelliteOrbit> orbits;
    std::ifstream file(filepath);
    std::string line;
    
    // Minimal SP3 parsing logic placeholder
    while (std::getline(file, line)) {
        if (line.empty() || line[0] != 'P') continue;
        std::stringstream ss(line);
        char type;
        int id;
        double x, y, z, clk;
        ss >> type >> id >> x >> y >> z >> clk;
        orbits.push_back({id, 0.0, Vector3(x * 1000.0, y * 1000.0, z * 1000.0), clk * 1e-6});
    }
    return orbits;
}

std::vector<SatelliteClock> ClkParser::parse(const std::string& filepath) {
    std::vector<SatelliteClock> clocks;
    std::ifstream file(filepath);
    std::string line;
    
    // Minimal CLK parsing logic placeholder
    while (std::getline(file, line)) {
        if (line.substr(0, 2) != "AS") continue;
        std::stringstream ss(line);
        std::string tag;
        int id;
        double t, bias;
        ss >> tag >> id >> t >> bias;
        clocks.push_back({id, t, bias});
    }
    return clocks;
}

} // namespace rtk
