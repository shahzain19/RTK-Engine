#include "rtk_engine/rtk_api.h"
#include "rtk_engine/rtk_solver.hpp"
#include "rtk_engine/config_manager.hpp"

extern "C" {

rtk_handle_t rtk_init(const char* config_path) {
    if (!rtk_engine::ConfigManager::instance().load(config_path)) {
        return nullptr;
    }
    // Assuming RTK_Solver is the main entry point class
    return new rtk::RTK_Solver(); 
}

int rtk_process_epoch(rtk_handle_t handle, const char* data) {
    auto* solver = static_cast<rtk::RTK_Solver*>(handle);
    if (!solver) return -1;
    // Assuming some process method exists
    // solver->process(data);
    return 0;
}

void rtk_shutdown(rtk_handle_t handle) {
    auto* solver = static_cast<rtk::RTK_Solver*>(handle);
    delete solver;
}

}
