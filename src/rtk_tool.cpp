/**
 * @file rtk_tool.cpp
 * @brief Production CLI tool for the RTK engine.
 */

#include "rtk_engine/rtk_app.hpp"
#include <iostream>
#include <csignal>
#include <memory>

namespace {
    std::unique_ptr<rtk::RtkEngineApp> g_app;
}

void handleSignal(int signum) {
    if (g_app) {
        std::cout << "\n[TOOL] Shutdown signal received. Stopping engine...\n";
        g_app->stop();
    }
}

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::string config_file = "config.toml";
    if (argc > 1) config_file = argv[1];

    std::cout << "========================================================================\n";
    std::cout << "        GEMINI-RTK: PROFESSIONAL NAVIGATION TERMINAL                     \n";
    std::cout << "========================================================================\n";
    
    try {
        g_app = std::make_unique<rtk::RtkEngineApp>();
        
        if (!g_app->start(config_file)) {
            std::cerr << "[TOOL] Engine failed to start.\n";
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "[TOOL] Critical Exception: " << e.what() << "\n";
        return 1;
    }

    std::cout << "[TOOL] Engine terminated.\n";
    return 0;
}
