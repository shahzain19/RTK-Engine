/**
 * @file nmea_formatter.hpp
 * @brief Formats RTK solution data into standard NMEA sentences.
 */

#ifndef RTK_ENGINE_OUTPUT_NMEA_FORMATTER_HPP
#define RTK_ENGINE_OUTPUT_NMEA_FORMATTER_HPP

#include "rtk_engine/common.hpp"
#include <string>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace rtk {

class NmeaFormatter {
public:
    /** @brief Formats an NMEA GGA sentence. */
    static std::string formatGGA(double t, const Vector3& enu, int num_sats) {
        // Simplified GGA format: $GPGGA,time,lat,lon,fix,sats,hdop,alt,M,geoid,M,age,station*cs
        
        // Convert time to HHMMSS
        int hh = static_cast<int>(t) / 3600;
        int mm = (static_cast<int>(t) % 3600) / 60;
        int ss = static_cast<int>(t) % 60;
        
        char buf[256];
        // Using dummy lat/lon for demo, should map from enu + base
        std::snprintf(buf, sizeof(buf), "GPGGA,%02d%02d%02d,3725.3220,N,12205.0415,W,1,%02d,1.0,%.2f,M,0.0,M,,", 
                      hh, mm, ss, num_sats, enu.z);
        
        return "$" + std::string(buf) + "*" + checksum(buf);
    }

private:
    static std::string checksum(const char* s) {
        uint8_t cs = 0;
        while (*s) cs ^= *s++;
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%02X", cs);
        return std::string(buf);
    }
};

} // namespace rtk

#endif // RTK_ENGINE_OUTPUT_NMEA_FORMATTER_HPP
