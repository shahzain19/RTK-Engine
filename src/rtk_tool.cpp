/**
 * @file rtk_tool.cpp
 * @brief Production CLI tool for the RTK engine.
 */

#include "rtk_engine/rtk_app.hpp"
#include <iostream>
#include <csignal>

namespace {
    rtk::RtkEngineApp* g_app_ptr = nullptr;
}

void handleSignal(int signum) {
    if (g_app_ptr) g_app_ptr->stop();
}

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::string config_file = "config.toml";
    if (argc > 1) config_file = argv[1];

    std::cout << "========================================================================\n";
    std::cout << "        GEMINI-RTK: PROFESSIONAL NAVIGATION TERMINAL                     \n";
    std::cout << "========================================================================\n";
    
    rtk::RtkEngineApp app;
    g_app_ptr = &app;
    
    if (!app.start(config_file)) {
        return 1;
    }

    return 0;
}
