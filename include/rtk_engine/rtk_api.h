#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle for the RTK engine instance
typedef void* rtk_handle_t;

// Initialize the RTK engine
rtk_handle_t rtk_init(const char* config_path);

// Process a single epoch of GNSS observations
int rtk_process_epoch(rtk_handle_t handle, const char* data);

// Shut down and clean up
void rtk_shutdown(rtk_handle_t handle);

#ifdef __cplusplus
}
#endif
