#include "rtk_engine/rtk_api.h"
#include "rtk_engine/rtk_app.hpp"
#include <cstring>

extern "C" {

rtk_handle_t rtk_init(const char* config_path) {
    auto* app = new rtk::RtkEngineApp();
    if (!app->start(config_path, true)) { // Start in background
        delete app;
        return nullptr;
    }
    return static_cast<rtk_handle_t>(app);
}

int rtk_feed_raw_data(rtk_handle_t handle, const uint8_t* buffer, int length) {
    auto* app = static_cast<rtk::RtkEngineApp*>(handle);
    if (!app) return -1;
    app->feedRoverData(buffer, length);
    return 0;
}

int rtk_inject_imu(rtk_handle_t handle, const rtk_imu_t* imu) {
    auto* app = static_cast<rtk::RtkEngineApp*>(handle);
    if (!app || !imu) return -1;
    app->injectImu(*imu);
    return 0;
}

bool rtk_get_solution(rtk_handle_t handle, rtk_solution_t* sol) {
    auto* app = static_cast<rtk::RtkEngineApp*>(handle);
    if (!app || !sol) return false;
    return app->getLatestSolution(*sol);
}

const char* rtk_get_version(void) {
    return "1.2.0-commercial";
}

void rtk_shutdown(rtk_handle_t handle) {
    auto* app = static_cast<rtk::RtkEngineApp*>(handle);
    if (app) {
        app->stop();
        delete app;
    }
}

}
