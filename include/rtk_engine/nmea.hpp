/**
 * @file nmea.hpp
 * @brief Robust parser for NMEA 0183 sentences.
 */

#ifndef RTK_ENGINE_NMEA_HPP
#define RTK_ENGINE_NMEA_HPP

#include "rtk_engine/common.hpp"
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>

namespace rtk {

/**
 * @brief Parses NMEA streams and extracts positioning data (GGA, RMC).
 * @details Handles stream fragmentation and performs checksum validation.
 */
class NmeaParser {
public:
    NmeaParser() = default;

    /**
     * @brief Ingests raw NMEA stream chunks.
     * @param stream_chunk New data from a serial port or network socket.
     * @return std::vector<NmeaData> List of fully parsed and validated sentences.
     */
    std::vector<NmeaData> parseStream(const std::string& stream_chunk) {
        buffer_ += stream_chunk;
        std::vector<NmeaData> results;

        size_t start_pos;
        while ((start_pos = buffer_.find('$')) != std::string::npos) {
            // Sentences end with CRLF (\r\n) or just LF (\n)
            size_t end_pos = buffer_.find("\n", start_pos);
            if (end_pos == std::string::npos) {
                // Fragmented/incomplete sentence.
                // Discard data before '$' to avoid buffer growth, then break.
                if (start_pos > 0) {
                    buffer_ = buffer_.substr(start_pos);
                }
                break;
            }

            // Extract the potential sentence
            std::string sentence = buffer_.substr(start_pos, end_pos - start_pos + 1);
            // Advance buffer past this sentence
            buffer_ = buffer_.substr(end_pos + 1);

            // Clean sentence (remove carriage returns or newlines from end)
            while (!sentence.empty() && (sentence.back() == '\r' || sentence.back() == '\n')) {
                sentence.pop_back();
            }

            NmeaData data = parseSentence(sentence);
            if (data.valid) {
                results.push_back(data);
            }
        }

        return results;
    }

    /**
     * @brief Explicitly validate NMEA 8-bit XOR checksum.
     * @param sentence Full NMEA sentence starting with '$' and including '*CS'.
     * @return true If checksum is valid.
     */
    static bool validateChecksum(const std::string& sentence) {
        if (sentence.empty() || sentence[0] != '$') {
            return false;
        }

        size_t star_idx = sentence.find('*');
        if (star_idx == std::string::npos || star_idx + 3 > sentence.length()) {
            return false;
        }

        // XOR payload between '$' and '*'
        unsigned char calculated_xor = 0;
        for (size_t i = 1; i < star_idx; ++i) {
            calculated_xor ^= static_cast<unsigned char>(sentence[i]);
        }

        std::string hex_cs = sentence.substr(star_idx + 1, 2);
        char* endptr;
        unsigned long parsed_cs = std::strtoul(hex_cs.c_str(), &endptr, 16);
        if (endptr == hex_cs.c_str()) {
            return false;
        }

        return calculated_xor == static_cast<unsigned char>(parsed_cs);
    }

    /**
     * @brief Parse a single validated NMEA sentence.
     * @param sentence Raw NMEA string.
     * @return NmeaData Populated data structure.
     */
    NmeaData parseSentence(const std::string& sentence) {
        NmeaData data;
        
        if (!validateChecksum(sentence)) {
            return data; // returns valid = false
        }

        // Find asterisks to split payload from checksum
        size_t star_idx = sentence.find('*');
        std::string payload = sentence.substr(1, star_idx - 1); // remove leading '$'

        // Split fields by comma (preserving empty fields)
        std::vector<std::string> fields;
        std::string field;
        std::stringstream ss(payload);
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }
        if (!payload.empty() && payload.back() == ',') {
            fields.push_back("");
        }

        if (fields.empty()) {
            return data;
        }

        data.sentence_type = fields[0];

        if (data.sentence_type == "GNGGA" || data.sentence_type == "GPGGA") {
            // GGA Fields:
            // 0: $..GGA
            // 1: UTC Time (hhmmss.ss)
            // 2: Latitude (ddmm.mmmmm)
            // 3: N/S Indicator
            // 4: Longitude (dddmm.mmmmm)
            // 5: E/W Indicator
            // 6: Position Fix Quality (0=invalid, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float)
            // 7: Satellites Used
            // 8: HDOP
            // 9: Altitude (MSL)
            // 10: Altitude Units (M)
            if (fields.size() > 9) {
                data.utc_time = fields[1];
                char ns = fields[3].empty() ? 'N' : fields[3][0];
                char ew = fields[5].empty() ? 'E' : fields[5][0];
                data.latitude = convertToDecimalDegrees(fields[2], ns, false);
                data.longitude = convertToDecimalDegrees(fields[4], ew, true);
                data.fix_quality = safeStoi(fields[6]);
                data.num_satellites = safeStoi(fields[7]);
                data.hdop = safeStod(fields[8], 99.9);
                data.altitude = safeStod(fields[9], 0.0);
                data.valid = true;
            }
        } else if (data.sentence_type == "GNRMC" || data.sentence_type == "GPRMC") {
            // RMC Fields:
            // 0: $..RMC
            // 1: UTC Time
            // 2: Status (A=Active, V=Void)
            // 3: Latitude
            // 4: N/S
            // 5: Longitude
            // 6: E/W
            if (fields.size() > 6) {
                data.utc_time = fields[1];
                data.rmc_active = (fields[2] == "A");
                char ns = fields[4].empty() ? 'N' : fields[4][0];
                char ew = fields[6].empty() ? 'E' : fields[6][0];
                data.latitude = convertToDecimalDegrees(fields[3], ns, false);
                data.longitude = convertToDecimalDegrees(fields[5], ew, true);
                data.valid = true;
            }
        }

        return data;
    }

private:
    std::string buffer_;

    /** @brief Convert DDMM.MMMMM/DDDMM.MMMMM to decimal degrees. */
    double convertToDecimalDegrees(const std::string& field, char nsew, bool is_longitude) {
        if (field.empty()) {
            return 0.0;
        }

        size_t dot = field.find('.');
        size_t deg_len = is_longitude ? 3 : 2;

        if (dot != std::string::npos && dot > deg_len) {
            deg_len = dot - 2;
        }

        if (field.length() <= deg_len) {
            return 0.0;
        }

        std::string deg_str = field.substr(0, deg_len);
        std::string min_str = field.substr(deg_len);

        double degrees = safeStod(deg_str);
        double minutes = safeStod(min_str);
        double decimal = degrees + (minutes / 60.0);

        if (nsew == 'S' || nsew == 's' || nsew == 'W' || nsew == 'w') {
            decimal = -decimal;
        }

        return decimal;
    }

    /** @brief Helper functions for exception-safe parsing. */
    static int safeStoi(const std::string& str, int default_val = 0) {
        if (str.empty()) return default_val;
        char* endptr;
        long val = std::strtol(str.c_str(), &endptr, 10);
        if (endptr == str.c_str()) return default_val;
        return static_cast<int>(val);
    }

    static double safeStod(const std::string& str, double default_val = 0.0) {
        if (str.empty()) return default_val;
        char* endptr;
        double val = std::strtod(str.c_str(), &endptr);
        if (endptr == str.c_str()) return default_val;
        return val;
    }
};

} // namespace rtk

#endif // RTK_ENGINE_NMEA_HPP
