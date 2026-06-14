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
        } catch (const std::exception& err) {
            std::cerr << "[CONFIG] Error loading " << filepath << ": " << err.what() << "\n";
            return false;
        }
    }

    template <typename T>
    T get(const std::string& key, T default_val = T{}) const {
        // Handle nested keys like "ntrip.host"
        size_t pos = key.find('.');
        if (pos != std::string::npos) {
            std::string parent = key.substr(0, pos);
            std::string child = key.substr(pos + 1);
            
            auto node = config_.get(parent);
            if (node && node->is_table()) {
                auto val = node->as_table()->get(child);
                if (val) return val->value_or(default_val);
            }
        } else {
            auto node = config_.get(key);
            if (node) return node->value_or(default_val);
        }
        
        return default_val;
    }

private:
    ConfigManager() = default;
    toml::table config_;
};

} // namespace rtk_engine
