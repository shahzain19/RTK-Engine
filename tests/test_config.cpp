#include <iostream>
#include <fstream>
#include "rtk_engine/config_manager.hpp"
#include <cassert>

int main() {
    // Create a temporary TOML file
    std::ofstream ofs("test.toml");
    ofs << "[physics]\nspeed_of_light = 100.0\n";
    ofs.close();

    rtk_engine::ConfigManager& cm = rtk_engine::ConfigManager::instance();
    if (cm.load("test.toml")) {
        double sol = cm.get<double>("physics.speed_of_light");
        std::cout << "Read speed_of_light: " << sol << std::endl;
        assert(sol == 100.0);
        std::cout << "ConfigManager test passed!" << std::endl;
    } else {
        std::cerr << "ConfigManager test failed!" << std::endl;
        return 1;
    }

    return 0;
}
