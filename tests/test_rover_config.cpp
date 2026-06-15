#include <iostream>
#include <fstream>
#include "rtk_engine/config_manager.hpp"
#include <cassert>

int main() {
    // Create a temporary TOML file
    std::ofstream ofs("test_rover.toml");
    ofs << "[rover]\nfile = \"test.obs\"\n";
    ofs.close();

    rtk_engine::ConfigManager& cm = rtk_engine::ConfigManager::instance();
    if (cm.load("test_rover.toml")) {
        std::string file = cm.get<std::string>("rover.file");
        std::cout << "Read rover file: " << file << std::endl;
        assert(file == "test.obs");
        std::cout << "Rover config test passed!" << std::endl;
    } else {
        std::cerr << "ConfigManager test failed!" << std::endl;
        return 1;
    }

    return 0;
}
