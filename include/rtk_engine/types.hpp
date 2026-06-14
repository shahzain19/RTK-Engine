/**
 * @file types.hpp
 * @brief Common data structures for GNSS observations and state.
 */

#ifndef RTK_ENGINE_TYPES_HPP
#define RTK_ENGINE_TYPES_HPP

#include "rtk_engine/math/vector3.hpp"
#include "rtk_engine/constants.hpp"
#include <string>
#include <vector>

namespace rtk {

/**
 * @brief Supported GNSS constellations.
 */
enum class Constellation {
    GPS,
    GLONASS,
    GALILEO,
    BEIDOU,
    UNKNOWN
};

/**
 * @brief Decoded NMEA data for standard position reporting.
 */
struct NmeaData {
    bool valid = false;        ///< Is the data valid
    std::string sentence_type; ///< e.g., "GGA", "RMC"
    std::string utc_time;      ///< Time of day (hhmmss.ss)
    double latitude = 0.0;     ///< Latitude in decimal degrees
    double longitude = 0.0;    ///< Longitude in decimal degrees
    int fix_quality = 0;       ///< 0=Invalid, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float
    int num_satellites = 0;    ///< Number of satellites in use
    double hdop = 99.9;        ///< Horizontal Dilution of Precision
    double altitude = 0.0;     ///< MSL altitude (meters)
    bool rmc_active = false;   ///< Status from RMC (Active/Void)
};

/**
 * @brief Observation data for a specific signal (frequency).
 */
struct SignalObs {
    double pseudorange = 0.0;    ///< Measured pseudorange (meters)
    double carrier_phase = 0.0;  ///< Measured carrier phase (cycles)
    double doppler = 0.0;        ///< Measured Doppler frequency (Hz)
    double snr = 0.0;            ///< Signal-to-Noise Ratio (dB-Hz)
    double frequency = 0.0;      ///< Actual frequency of the signal (Hz)
};

/**
 * @brief Comprehensive observations for a single satellite.
 */
struct SatelliteObs {
    int svid = 0;                ///< Satellite ID / PRN
    Constellation sys = Constellation::GPS; ///< GNSS constellation
    double elevation = 0.0;      ///< Elevation angle (radians)
    double azimuth = 0.0;        ///< Azimuth angle (radians)
    
    /** @brief Modern multi-frequency signals. */
    std::vector<SignalObs> signals;

    /** @name Legacy compatibility fields (Phase 1/2 single-freq logic) */
    ///@{
    double pseudorange = 0.0;    
    double carrier_phase = 0.0;
    ///@}
};

/**
 * @brief Set of satellite observations for a specific epoch.
 */
struct EpochObs {
    double gps_time = 0.0;       ///< GPS Time of Week (seconds)
    int station_id = 0;          ///< Station ID (0 for Rover, 1 for Base)
    Vector3 ref_pos;             ///< Known reference position (ECEF meters)
    std::vector<SatelliteObs> sat_obs; ///< List of satellite observations
};

/**
 * @brief Inertial measurement data (IMU).
 */
struct ImuMeas {
    double t;                  ///< Measurement time (seconds)
    Vector3 acc;               ///< Acceleration (m/s^2) in body frame (includes gravity)
    Vector3 gyro;              ///< Angular rate (rad/s) in body frame
};

} // namespace rtk

#endif // RTK_ENGINE_TYPES_HPP
