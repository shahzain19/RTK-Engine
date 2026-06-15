/**
 * @file rinex_writer.hpp
 * @brief Writer for recording GNSS data in RINEX format.
 */

#ifndef RTK_ENGINE_OUTPUT_RINEX_WRITER_HPP
#define RTK_ENGINE_OUTPUT_RINEX_WRITER_HPP

#include "rtk_engine/common.hpp"
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>

namespace rtk {

class RinexWriter {
public:
    RinexWriter(const std::string& filename) : filename_(filename) {
        file_.open(filename_, std::ios::out);
    }
    
    ~RinexWriter() {
        if (file_.is_open()) file_.close();
    }

    bool isOpen() const { return file_.is_open(); }

    /** @brief Writes a simple observation epoch. */
    void writeEpoch(double t, const std::vector<SatelliteObs>& obs) {
        if (!file_.is_open()) return;

        // Simplified RINEX 3.x epoch format
        // > YYYY MM DD HH MM SS.sssss flag num_sats
        std::time_t rawtime = static_cast<std::time_t>(t);
        std::tm* timeinfo = std::gmtime(&rawtime);
        
        file_ << "> " 
              << std::setw(4) << timeinfo->tm_year + 1900 << " "
              << std::setw(2) << timeinfo->tm_mon + 1 << " "
              << std::setw(2) << timeinfo->tm_mday << " "
              << std::setw(2) << timeinfo->tm_hour << " "
              << std::setw(2) << timeinfo->tm_min << " "
              << std::fixed << std::setprecision(4) << std::setw(11) << (t - static_cast<int>(t)) + timeinfo->tm_sec
              << " 0 " << std::setw(2) << obs.size() << "\n";

        for (const auto& s : obs) {
            file_ << "G" << std::setw(2) << s.svid << " " 
                  << std::fixed << std::setprecision(3) << std::setw(14) << s.pseudorange << "\n";
        }
    }

private:
    std::string filename_;
    std::ofstream file_;
};

} // namespace rtk

#endif // RTK_ENGINE_OUTPUT_RINEX_WRITER_HPP
