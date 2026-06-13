/**
 * @file messages.hpp
 * @brief Data structures for decoded RTCM v3 messages.
 */

#ifndef RTK_ENGINE_RTCM3_MESSAGES_HPP
#define RTK_ENGINE_RTCM3_MESSAGES_HPP

#include "rtk_engine/types.hpp"
#include "rtk_engine/orbit.hpp"
#include <vector>
#include <cstdint>

namespace rtk {

/** @brief RTCM 1005: Stationary RTK Reference Station Antenna Reference Point (ARP). */
struct Msg1005 {
    uint16_t station_id;
    Vector3 antenna_ecef; ///< Reference coordinates in ECEF meters.
};

/** @brief Satellite-specific data for RTCM 1004. */
struct Msg1004Sat {
    uint8_t svid;
    double pseudorange_l1;    ///< Full L1 pseudorange (meters).
    double carrier_phase_l1;  ///< L1 carrier phase (cycles).
    uint8_t lock_time_l1;
    uint8_t cnr_l1;           ///< L1 Carrier-to-Noise Ratio (dB-Hz).
    double pseudorange_l2;    ///< Full L2 pseudorange (meters).
    double carrier_phase_l2;  ///< L2 carrier phase (cycles).
    uint8_t lock_time_l2;
    uint8_t cnr_l2;           ///< L2 Carrier-to-Noise Ratio (dB-Hz).
};

/** @brief RTCM 1004: Extended GPS L1/L2 Observables. */
struct Msg1004 {
    uint16_t station_id;
    uint32_t gps_tow_ms;      ///< GPS Time of Week (milliseconds).
    std::vector<Msg1004Sat> sats;
};

/** @brief RTCM 1019: GPS Ephemeris. */
struct Msg1019 {
    GpsEphemeris eph;
};

/** @brief RTCM 1020: GLONASS Ephemeris. */
struct Msg1020 {
    GloEphemeris eph;
};

// --- MSM (Multiple Signal Messages) structures ---

/** @brief Header common to all MSM (107x-112x) messages. */
struct MsmHeader {
    uint16_t message_type;
    uint16_t station_id;
    uint32_t tow_ms;
    bool sync_flag;
    uint8_t multiple_msg_flag;
    uint8_t iodes;
    uint8_t session_info;
    uint8_t clock_steering;
    uint8_t external_clock;
    uint8_t smoothing_ind;
    uint8_t smoothing_int;
    uint64_t sat_mask;   ///< Satellite presence mask (bits 1-64).
    uint32_t sig_mask;   ///< Signal presence mask (bits 1-32).
    std::vector<bool> cell_mask; ///< Signal/Satellite combination mask.
};

/** @brief Satellite-specific data for MSM. */
struct MsmSatData {
    uint8_t svid;
    uint16_t rough_range_ms;
    uint8_t rough_range_fine;
    double rough_range;      ///< Reconstructed rough range in meters.
};

/** @brief Signal-specific data for MSM. */
struct MsmSigData {
    double pseudorange;      ///< Full pseudorange in meters.
    double carrier_phase;    ///< Full carrier phase in cycles.
    uint16_t lock_time;
    uint8_t snr;             ///< Signal-to-Noise Ratio.
    bool half_cycle_amb;
};

/** @brief Container for a full MSM message. */
struct MsmMessage {
    MsmHeader header;
    std::vector<MsmSatData> sats;
    std::vector<MsmSigData> cells; ///< List of signal data matching the cell mask.
};

} // namespace rtk

#endif // RTK_ENGINE_RTCM3_MESSAGES_HPP
