/**
 * @file ntrip_client.hpp
 * @brief NTRIP client for real-time GNSS correction streaming.
 */

#ifndef RTK_ENGINE_NTRIP_CLIENT_HPP
#define RTK_ENGINE_NTRIP_CLIENT_HPP

#include "rtk_engine/common.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>

namespace rtk {

/**
 * @brief Network client to stream RTCM corrections from an NTRIP caster.
 */
class NtripClient {
public:
    /** @brief NTRIP Connection parameters. */
    struct Config {
        std::string host;
        int port = 2101;
        std::string mountpoint;
        std::string user;
        std::string password;
    };

    NtripClient(const Config& config) : config_(config), socket_fd_(-1) {}

    ~NtripClient() {
        disconnect();
    }

    /**
     * @brief Performs TCP handshake and NTRIP GET authorization.
     * @return true If connected and authorized.
     */
    bool connect() {
        struct hostent* server = gethostbyname(config_.host.c_str());
        if (!server) {
            std::cerr << "[NTRIP] Host lookup failed: " << config_.host << "\n";
            return false;
        }

        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) return false;

        struct sockaddr_in serv_addr;
        std::memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        std::memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
        serv_addr.sin_port = htons(config_.port);

        if (::connect(socket_fd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            std::cerr << "[NTRIP] Connection failed to " << config_.host << ":" << config_.port << "\n";
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // Send NTRIP GET Request
        std::string auth = base64Encode(config_.user + ":" + config_.password);
        std::string request = 
            "GET /" + config_.mountpoint + " HTTP/1.0\r\n" +
            "User-Agent: NTRIP GeminiRTK/1.0\r\n" +
            "Authorization: Basic " + auth + "\r\n" +
            "Connection: close\r\n\r\n";

        if (send(socket_fd_, request.c_str(), request.length(), 0) < 0) {
            disconnect();
            return false;
        }

        // Wait for ICY 200 OK (the standard NTRIP success response)
        char buffer[1024];
        int n = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) {
            disconnect();
            return false;
        }
        buffer[n] = '\0';
        std::string response(buffer);
        if (response.find("ICY 200 OK") == std::string::npos && response.find("HTTP/1.0 200 OK") == std::string::npos) {
            std::cerr << "[NTRIP] Server rejected request: " << response << "\n";
            disconnect();
            return false;
        }

        std::cout << "[NTRIP] Connected to " << config_.host << "/" << config_.mountpoint << "\n";
        return true;
    }

    /** @brief Closes the network connection. */
    void disconnect() {
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
    }

    /**
     * @brief Read available data from the stream.
     * @param buffer Byte array to store received data.
     * @param max_len Maximum bytes to read.
     * @return int Number of bytes read, or -1 on error.
     */
    int read(uint8_t* buffer, size_t max_len) {
        if (socket_fd_ < 0) return -1;
        return recv(socket_fd_, buffer, max_len, 0);
    }

private:
    Config config_;
    int socket_fd_;

    /** @brief RFC 4648 Base64 encoding for Basic Auth. */
    static std::string base64Encode(const std::string& in) {
        static const char lookup[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string out;
        int val = 0, valb = -6;
        for (unsigned char c : in) {
            val = (val << 8) + c;
            valb += 8;
            while (valb >= 0) {
                out.push_back(lookup[(val >> valb) & 0x3F]);
                valb -= 6;
            }
        }
        if (valb > -6) out.push_back(lookup[((val << 8) >> (valb + 8)) & 0x3F]);
        while (out.size() % 4) out.push_back('=');
        return out;
    }
};

} // namespace rtk

#endif // RTK_ENGINE_NTRIP_CLIENT_HPP
