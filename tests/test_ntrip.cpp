#include "rtk_engine/ntrip_client.hpp"
#include <iostream>

int main() {
    rtk::NtripClient::Config config;
    config.host = "rtk2go.com"; // Common NTRIP caster
    config.port = 2101;
    config.mountpoint = "TEST";
    config.user = "user";
    config.password = "password";

    rtk::NtripClient client(config);
    if (client.connect()) {
        std::cout << "Successfully connected!" << std::endl;
        uint8_t buffer[1024];
        int n = client.read(buffer, sizeof(buffer));
        if (n > 0) {
            std::cout << "Read " << n << " bytes" << std::endl;
        }
        client.disconnect();
    } else {
        std::cout << "Failed to connect." << std::endl;
    }
    return 0;
}
