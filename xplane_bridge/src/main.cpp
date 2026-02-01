#include <iostream>
#include <cstdint>
#include <string>

#include "udp_rx.hpp"
#include "xplane_decoder.hpp"

int main() {
    const uint16_t port = 49000;

    UdpReceiver rx;
    if(!rx.open(port)){
        std::cerr << "Failed to open UDP receiver on port " << port << "\n";
        return 1;
    }

    XPlaneDataDecoder decoder;

    std::cout << "Listening on UDP port " << port << "...\n";

    while (true) {
        std::string ip;
        uint16_t src_port = 0;

        auto pkt = rx.recv_packet(&ip, &src_port);
        if (pkt.empty())    
        {
            continue;
        }

        std::cout << "Got " << pkt.size() << "  bytes from " << ip << ":" << src_port<< "\n";

        auto state = decoder.decode(pkt);
        if (state.has_value()) {
    // Print a single clean line of the values we care about.
            std::cout
                << "KIAS=" << state->kias
                << " ALT(ft)=" << state->alt_msl_ft
                << " PITCH=" << state->pitch_deg
                << " ROLL=" << state->roll_deg
                << " HDG=" << state->hdg_deg
                << "\n";
        }
    }

    return 0;

}