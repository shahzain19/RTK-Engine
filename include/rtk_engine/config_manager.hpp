#pragma once

#include <string>
#include <iostream>
#include "external/toml.hpp"

namespace rtk_engine {

class ConfigManager {
public:
    static ConfigManager& instance() {
        static ConfigManager instance;
        return instance;
    }

    bool load(const std::string& filepath) {
        try {
            config_ = toml::parse_file(filepath);
            return true;
        } catch (const toml::parse_error& err) {
            std::cerr << "Parsing failed: " << err << "\n";
            return false;
        }
    }

    template <typename T>
    T get(const std::string& key) const {
        auto node = config_[key];
        if (node) {
            return node.value_or(T{});
        }
        
        // Handle nested keys (basic implementation)
        size_t pos = key.find('.');
        if (pos != std::string::npos) {
            std::string parent = key.substr(0, pos);
            std::string child = key.substr(pos + 1);
            if (config_[parent].is_table()) {
                return config_[parent].as_table()->get(child)->value_or(T{});
            }
        }
        return T{};
    }

private:
    ConfigManager() = default;
    toml::table config_;
};

} // namespace rtk_engine
