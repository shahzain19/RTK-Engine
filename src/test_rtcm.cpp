/**
 * @file test_rtcm.cpp
 * @brief Unit tests for the RTCM v3 decoder module.
 */

#include "rtk_engine/common.hpp"
#include "rtk_engine/rtcm3.hpp"
#include <iostream>
#include <vector>
#include <cassert>
#include <iomanip>

/**
 * @brief Helper to append 24-bit RTCM CRC to a byte buffer.
 */
void appendCrc24q(std::vector<uint8_t>& data) {
    uint32_t crc = 0;
    for (size_t i = 0; i < data.size(); ++i) {
        crc ^= (static_cast<uint32_t>(data[i]) << 16);
        for (int j = 0; j < 8; ++j) {
            crc <<= 1;
            if (crc & 0x01000000) {
                crc ^= 0x1864CFB;
            }
        }
    }
    data.push_back(static_cast<uint8_t>((crc >> 16) & 0xFF));
    data.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(crc & 0xFF));
}

int main() {
    std::cout << "Starting RTCM3 Parser Test...\n";

    // --- TEST 1: Message 1005 (ARP) Generation & Parsing ---
    std::vector<uint8_t> payload = {
        0x3E, 0xD4, 0xD2, 0x00, 0x00, // Type 1005, Station 1234
        0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    };
    
    uint16_t len = static_cast<uint16_t>(payload.size());
    std::vector<uint8_t> frame = {
        0xD3, 
        static_cast<uint8_t>((len >> 8) & 0x03), 
        static_cast<uint8_t>(len & 0xFF)
    };
    frame.insert(frame.end(), payload.begin(), payload.end());
    appendCrc24q(frame);

    rtk::Rtcm3Parser parser;
    auto results = parser.parseStream(frame.data(), frame.size());

    assert(results.size() == 1);
    assert(results[0].message_type == 1005);
    assert(results[0].crc_valid == true);

    rtk::Msg1005 msg1005;
    bool ok = rtk::Rtcm3Parser::decode1005(results[0].payload, msg1005);
    assert(ok);
    assert(msg1005.station_id == 1234);

    std::cout << "TEST 1 PASSED: Message 1005 parsed correctly.\n";

    // --- TEST 2: Multi-frame Stream & Fragmentation Handling ---
    std::vector<uint8_t> partial1(frame.begin(), frame.begin() + 10);
    std::vector<uint8_t> partial2(frame.begin() + 10, frame.end());

    auto res1 = parser.parseStream(partial1.data(), partial1.size());
    assert(res1.empty()); // Should wait for more data

    auto res2 = parser.parseStream(partial2.data(), partial2.size());
    assert(res2.size() == 1);
    assert(res2[0].message_type == 1005);

    std::cout << "TEST 2 PASSED: Fragmented stream handled correctly.\n";

    std::cout << "All RTCM3 Parser Tests PASSED!\n";
    return 0;
}
