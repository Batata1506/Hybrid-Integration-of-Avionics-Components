#pragma once
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>

#include "flight_state.hpp"

class FlightStateUdpPublisher {
public:
    FlightStateUdpPublisher(const char* ip = "127.0.0.1", uint16_t port = 5555)
    {
        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);

        std::memset(&addr_, 0, sizeof(addr_));
        addr_.sin_family = AF_INET;
        addr_.sin_port   = htons(port);
        ::inet_pton(AF_INET, ip, &addr_.sin_addr);
    }

    ~FlightStateUdpPublisher()
    {
        if (sock_ >= 0) ::close(sock_);
    }

    void publish(const FlightState& s)
    {
        if (sock_ < 0) return;

        // Build compact JSON
        std::ostringstream os;
        os << std::fixed << std::setprecision(2)
        << "{"
        << "\"kias\":" << s.kias << ","
        << "\"ktas\":" << s.ktas << ","
        << "\"ktgs\":" << s.ktgs << ","
        << "\"pitch_deg\":" << s.pitch_deg << ","
        << "\"roll_deg\":" << s.roll_deg << ","
        << "\"hdg_deg\":" << s.hdg_deg << ","
        << "\"alt_msl_ft\":" << s.alt_msl_ft << ","
        << "\"seq_att\":" << s.seq_att << ","
        << "\"seq_spd\":" << s.seq_spd << ","
        << "\"seq_alt\":" << s.seq_alt << ","
        << "\"tx_time_us_att\":" << s.tx_time_us_att << ","
        << "\"tx_time_us_spd\":" << s.tx_time_us_spd << ","
        << "\"tx_time_us_alt\":" << s.tx_time_us_alt
        << "}";


        const std::string msg = os.str() + "\n";
        ::sendto(sock_, msg.data(), msg.size(), 0,
                 reinterpret_cast<const sockaddr*>(&addr_), sizeof(addr_));
    }

private:
    int sock_{-1};
    sockaddr_in addr_{};
};