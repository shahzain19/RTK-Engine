/**
 * @file rtcm3.hpp
 * @brief RTCM v3.x protocol parser and decoder.
 */

#ifndef RTK_ENGINE_RTCM3_HPP
#define RTK_ENGINE_RTCM3_HPP

#include "rtk_engine/common.hpp"
#include "rtk_engine/orbit.hpp"
#include "rtk_engine/rtcm3/bit_reader.hpp"
#include "rtk_engine/rtcm3/messages.hpp"
#include <vector>
#include <cstdint>
#include <iostream>

namespace rtk {

/**
 * @brief Handles RTCM v3.x framing, CRC validation, and message decoding.
 * @details Supports legacy GPS/GLONASS messages and modern MSM formats.
 */
class Rtcm3Parser {
public:
    static constexpr uint8_t PREAMBLE = 0xD3; ///< RTCM3 frame preamble

    /** @brief Encapsulates a single extracted RTCM3 frame. */
    struct Frame {
        uint16_t message_type;        ///< e.g., 1004, 1005, 1019, 1074...
        std::vector<uint8_t> payload; ///< Message-specific payload
        bool crc_valid;               ///< Result of CRC-24Q check
    };

    Rtcm3Parser() = default;

    /**
     * @brief Ingest binary data and extract valid RTCM3 frames.
     * @param data Raw byte stream.
     * @param len Number of bytes.
     * @return std::vector<Frame> List of successfully extracted frames.
     */
    std::vector<Frame> parseStream(const uint8_t* data, size_t len) {
        std::vector<Frame> frames;
        buffer_.insert(buffer_.end(), data, data + len);

        while (buffer_.size() >= 3) {
            if (buffer_[0] != PREAMBLE) {
                buffer_.erase(buffer_.begin());
                continue;
            }

            uint16_t length = ((static_cast<uint16_t>(buffer_[1]) & 0x03) << 8) | buffer_[2];
            size_t total_frame_size = 3 + length + 3;

            if (buffer_.size() < total_frame_size) {
                break;
            }

            std::vector<uint8_t> full_frame(buffer_.begin(), buffer_.begin() + total_frame_size);
            
            Frame frame;
            frame.crc_valid = validateCrc(full_frame.data(), total_frame_size);
            
            if (frame.crc_valid) {
                frame.payload.assign(buffer_.begin() + 3, buffer_.begin() + 3 + length);
                if (length >= 2) {
                    frame.message_type = (static_cast<uint16_t>(frame.payload[0]) << 4) | (frame.payload[1] >> 4);
                }
                frames.push_back(std::move(frame));
            }

            buffer_.erase(buffer_.begin(), buffer_.begin() + total_frame_size);
        }

        return frames;
    }

    /**
     * @brief CRC-24Q parity algorithm implementation.
     * @param data Full frame data including preamble and length.
     * @param len Length of the data.
     * @return true If CRC is valid (remainder is zero).
     */
    static bool validateCrc(const uint8_t* data, size_t len) {
        if (len < 3) return false;
        uint32_t crc = 0;
        for (size_t i = 0; i < len; ++i) {
            crc ^= (static_cast<uint32_t>(data[i]) << 16);
            for (int j = 0; j < 8; ++j) {
                crc <<= 1;
                if (crc & 0x01000000) crc ^= 0x1864CFB;
            }
        }
        return (crc & 0xFFFFFF) == 0;
    }

    /** @name Legacy Message Decoders */
    ///@{
    
    /** @brief Decode Message 1005 (Stationary RTK Reference Station ARP). */
    static bool decode1005(const std::vector<uint8_t>& payload, Msg1005& out) {
        if (payload.size() < 19) return false;
        BitReader br(payload.data(), payload.size());
        uint16_t type = br.read(12);
        if (type != 1005) return false;

        out.station_id = br.read(12);
        br.skip(6); // Reserved
        br.skip(4); // Indicator

        out.antenna_ecef.x = static_cast<double>(readSigned64(br, 38)) * 0.0001;
        out.antenna_ecef.y = static_cast<double>(readSigned64(br, 38)) * 0.0001;
        out.antenna_ecef.z = static_cast<double>(readSigned64(br, 38)) * 0.0001;

        return true;
    }

    /** @brief Decode Message 1004 (Extended GPS L1/L2 Observables). */
    static bool decode1004(const std::vector<uint8_t>& payload, Msg1004& out) {
        if (payload.size() < 8) return false;
        BitReader br(payload.data(), payload.size());
        uint16_t type = br.read(12);
        if (type != 1004) return false;

        out.station_id = br.read(12);
        out.gps_tow_ms = br.read(30);
        br.skip(1);
        uint8_t num_sats = br.read(5);
        br.skip(1);
        br.skip(3);

        for (int i = 0; i < num_sats; ++i) {
            Msg1004Sat sat;
            sat.svid = br.read(6);
            br.skip(1);
            uint32_t pr_l1_raw = br.read(24);
            int32_t cp_pr_l1_raw = br.readSigned(20);
            sat.lock_time_l1 = br.read(7);
            uint8_t amb_l1 = br.read(8);
            sat.cnr_l1 = br.read(8) * 0.25;

            br.skip(2);
            int32_t pr_diff_l2_l1 = br.readSigned(14);
            int32_t cp_pr_l2_raw = br.readSigned(20);
            sat.lock_time_l2 = br.read(7);
            sat.cnr_l2 = br.read(8) * 0.25;

            double pr_l1 = pr_l1_raw * 0.02 + amb_l1 * SPEED_OF_LIGHT / 1000.0;
            sat.pseudorange_l1 = pr_l1;
            sat.carrier_phase_l1 = (pr_l1 + cp_pr_l1_raw * 0.0005) / GPS_L1_WAVELENGTH;
            
            double pr_l2 = pr_l1 + pr_diff_l2_l1 * 0.02;
            sat.pseudorange_l2 = pr_l2;
            constexpr double GPS_L2_WAVELENGTH = SPEED_OF_LIGHT / 1227.60e6;
            sat.carrier_phase_l2 = (pr_l1 + cp_pr_l2_raw * 0.0005) / GPS_L2_WAVELENGTH;

            out.sats.push_back(sat);
        }

        return true;
    }

    /** @brief Decode Message 1019 (GPS Ephemeris). */
    static bool decode1019(const std::vector<uint8_t>& payload, Msg1019& out) {
        if (payload.size() < 61) return false;
        BitReader br(payload.data(), payload.size());
        uint16_t type = br.read(12);
        if (type != 1019) return false;

        out.eph.svid = br.read(6);
        br.read(10); // Week number
        br.read(4);  // SV Accuracy
        br.read(2);  // Code on L2
        out.eph.idot = br.readSigned(14) * std::pow(2, -43) * M_PI;
        br.read(8);  // IODE
        out.eph.t_oc = br.read(16) * std::pow(2, 4);
        out.eph.af2 = br.readSigned(8) * std::pow(2, -55);
        out.eph.af1 = br.readSigned(16) * std::pow(2, -43);
        out.eph.af0 = br.readSigned(22) * std::pow(2, -31);
        br.read(10); // IODC
        out.eph.crs = br.readSigned(16) * std::pow(2, -5);
        out.eph.delta_n = br.readSigned(16) * std::pow(2, -43) * M_PI;
        out.eph.m0 = br.readSigned(32) * std::pow(2, -31) * M_PI;
        out.eph.cuc = br.readSigned(16) * std::pow(2, -29);
        out.eph.e = br.read(32) * std::pow(2, -33);
        out.eph.cus = br.readSigned(16) * std::pow(2, -29);
        out.eph.sqrt_a = br.read(32) * std::pow(2, -19);
        out.eph.toe = br.read(16) * std::pow(2, 4);
        out.eph.cic = br.readSigned(16) * std::pow(2, -29);
        out.eph.omg0 = br.readSigned(32) * std::pow(2, -31) * M_PI;
        out.eph.cis = br.readSigned(16) * std::pow(2, -29);
        out.eph.i0 = br.readSigned(32) * std::pow(2, -31) * M_PI;
        out.eph.crc = br.readSigned(16) * std::pow(2, -5);
        out.eph.omega = br.readSigned(32) * std::pow(2, -31) * M_PI;
        out.eph.omg_dot = br.readSigned(24) * std::pow(2, -43) * M_PI;
        out.eph.tgd = br.readSigned(8) * std::pow(2, -31);
        
        return true;
    }

    /** @brief Decode Message 1020 (GLONASS Ephemeris). */
    static bool decode1020(const std::vector<uint8_t>& payload, Msg1020& out) {
        if (payload.size() < 42) return false;
        BitReader br(payload.data(), payload.size());
        uint16_t type = br.read(12);
        if (type != 1020) return false;

        out.eph.svid = br.read(6);
        br.read(2); // freq number
        br.read(5); // reserved
        out.eph.toe = br.read(12) * 900.0; // sec of day
        
        // State vector (km, km/s, km/s^2)
        out.eph.pos.x = br.readSigned(24) * std::pow(2, -11);
        out.eph.vel.x = br.readSigned(24) * std::pow(2, -20);
        out.eph.acc.x = br.readSigned(5)  * std::pow(2, -30);
        
        out.eph.pos.y = br.readSigned(24) * std::pow(2, -11);
        out.eph.vel.y = br.readSigned(24) * std::pow(2, -20);
        out.eph.acc.y = br.readSigned(5)  * std::pow(2, -30);
        
        out.eph.pos.z = br.readSigned(24) * std::pow(2, -11);
        out.eph.vel.z = br.readSigned(24) * std::pow(2, -20);
        out.eph.acc.z = br.readSigned(5)  * std::pow(2, -30);
        
        out.eph.tau_n = br.readSigned(22) * std::pow(2, -30);
        out.eph.gamma_n = br.readSigned(11) * std::pow(2, -40);
        
        return true;
    }
    ///@}

    /** @name MSM Message Decoders */
    ///@{
    
    /** @brief Decode MSM Header and Masks. */
    static bool decodeMsmHeader(BitReader& br, MsmHeader& out) {
        out.message_type = br.read(12);
        out.station_id = br.read(12);
        out.tow_ms = br.read(30);
        out.sync_flag = br.read(1);
        out.multiple_msg_flag = br.read(3);
        out.iodes = br.read(7);
        out.session_info = br.read(2);
        out.clock_steering = br.read(2);
        out.external_clock = br.read(2);
        out.smoothing_ind = br.read(1);
        out.smoothing_int = br.read(3);

        // Satellite Mask (64 bits)
        uint64_t m1 = br.read(32);
        uint64_t m2 = br.read(32);
        out.sat_mask = (m1 << 32) | m2;

        // Signal Mask (32 bits)
        out.sig_mask = br.read(32);

        // Cell Mask
        int num_sats = 0;
        for (int i = 0; i < 64; ++i) if ((out.sat_mask >> (63 - i)) & 1) num_sats++;
        
        int num_sigs = 0;
        for (int i = 0; i < 32; ++i) if ((out.sig_mask >> (31 - i)) & 1) num_sigs++;

        int total_cells = num_sats * num_sigs;
        out.cell_mask.resize(total_cells);
        for (int i = 0; i < total_cells; ++i) out.cell_mask[i] = br.read(1);

        return true;
    }

    /** @brief Decode MSM4 (Multiple Signal Message 4). */
    static bool decodeMsm4(const std::vector<uint8_t>& payload, MsmMessage& out) {
        BitReader br(payload.data(), payload.size());
        if (!decodeMsmHeader(br, out.header)) return false;

        int num_sats = 0;
        std::vector<int> svids;
        for (int i = 0; i < 64; ++i) {
            if ((out.header.sat_mask >> (63 - i)) & 1) {
                num_sats++;
                svids.push_back(i + 1);
            }
        }

        // Sat Data
        for (int i = 0; i < num_sats; ++i) {
            MsmSatData sat;
            sat.svid = svids[i];
            sat.rough_range_ms = br.read(8);
            sat.rough_range_fine = br.read(10);
            sat.rough_range = (sat.rough_range_ms + sat.rough_range_fine / 1024.0) * SPEED_OF_LIGHT / 1000.0;
            out.sats.push_back(sat);
        }

        // Signal Data (MSM4: 15 bits PR, 22 bits CP)
        int num_sigs = 0;
        for (int i = 0; i < 32; ++i) if ((out.header.sig_mask >> (31 - i)) & 1) num_sigs++;

        int cell_idx = 0;
        for (int i = 0; i < num_sats; ++i) {
            for (int j = 0; j < num_sigs; ++j) {
                if (out.header.cell_mask[cell_idx++]) {
                    MsmSigData sig;
                    int32_t pr_fine = br.readSigned(15); // Fine PR in ms
                    int32_t cp_fine = br.readSigned(22); // Fine CP in ms
                    sig.lock_time = br.read(4);
                    br.read(1); // Half-cycle ambiguity
                    sig.snr = br.read(6);

                    sig.pseudorange = out.sats[i].rough_range + pr_fine * SPEED_OF_LIGHT / (1024.0 * 512.0 * 1000.0);
                    // For now, assume L1 wavelength for carrier phase conversion
                    sig.carrier_phase = (out.sats[i].rough_range + cp_fine * SPEED_OF_LIGHT / (1024.0 * 512.0 * 1000.0)) / GPS_L1_WAVELENGTH;
                    
                    out.cells.push_back(sig);
                }
            }
        }

        return true;
    }
    ///@}

private:
    std::vector<uint8_t> buffer_;
};

} // namespace rtk

#endif // RTK_ENGINE_RTCM3_HPP
