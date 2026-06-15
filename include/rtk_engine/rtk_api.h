#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct rtk_solution_t
 * @brief High-precision solution data output.
 */
typedef struct {
    double timestamp;      // GPS Time of Week (s)
    double lat, lon, alt;  // WGS84 coordinates (deg, m)
    float roll, pitch, yaw;// Euler angles (deg)
    float vn, ve, vu;      // Velocity North, East, Up (m/s)
    uint8_t fix_type;      // 0=None, 1=Single, 2=DGPS, 4=Fixed, 5=Float
    uint8_t num_sats;      // Number of satellites used
    float h_acc, v_acc;    // Horizontal and Vertical accuracy (m)
} rtk_solution_t;

/**
 * @struct rtk_imu_t
 * @brief Raw IMU measurement for injection.
 */
typedef struct {
    double timestamp;      // Measurement time (s)
    float acc[3];          // Accelerometer X, Y, Z (m/s^2)
    float gyro[3];         // Gyroscope X, Y, Z (rad/s)
} rtk_imu_t;

// Opaque handle for the RTK engine instance
typedef void* rtk_handle_t;

/**
 * @brief Initialize the RTK engine.
 * @return Opaque handle or NULL on failure.
 */
rtk_handle_t rtk_init(const char* config_path);

/**
 * @brief Feed raw serial/NTRIP data (RTCM3/NMEA) into the engine.
 */
int rtk_feed_raw_data(rtk_handle_t handle, const uint8_t* buffer, int length);

/**
 * @brief Inject a synchronized IMU measurement.
 */
int rtk_inject_imu(rtk_handle_t handle, const rtk_imu_t* imu);

/**
 * @brief Get the latest calculated solution.
 * @return true if a new solution was retrieved.
 */
bool rtk_get_solution(rtk_handle_t handle, rtk_solution_t* sol);

/**
 * @brief Return the engine version string.
 */
const char* rtk_get_version(void);

/**
 * @brief Shut down and release resources.
 */
void rtk_shutdown(rtk_handle_t handle);

#ifdef __cplusplus
}
#endif
