#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>

#include "udp_rx.hpp"
#include <ed247.h>

int main() {
    const uint16_t node_red_port = 6001;

    UdpReceiver rx;
    if (!rx.open(node_red_port)) {
        std::cerr << "failed to open UDP on port " << node_red_port << "\n";
        return 1;
    }

    std::cout << "Listening for Node-RED UDP on 0.0.0.0:" << node_red_port << " ...\n";

    ed247_context_t ctx = nullptr;

    ed247_status_t st = ed247_load_file("ECIC.xml", &ctx);
    if (st != ED247_STATUS_SUCCESS) {
        std::cerr << "ed247_load_file failed, status=" << st << "\n";
        return 1;
    }

    ed247_stream_t s_ctrl = nullptr;

    st = ed247_find_stream(ctx, "VL2001_CTRL", &s_ctrl);  // or ed247_get_stream if yours supports it
    if (st != ED247_STATUS_SUCCESS || s_ctrl == nullptr) {
        std::cerr << "Could not find VL2001_CTRL stream\n";
        ed247_unload(ctx);
        return 1;
    }

    while (true) {
        std::string ip;
        uint16_t src_port = 0;

        std::vector<uint8_t> pkt = rx.recv_packet(&ip, &src_port);

        if (pkt.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        std::string payload(pkt.begin(), pkt.end());

        std::string command;
        auto pos = payload.find("\"name\":\"");
        if (pos != std::string::npos) {
            pos += 8;
            auto end = payload.find("\"", pos);
            if (end != std::string::npos) {
                command = payload.substr(pos, end - pos);
            }
        }

        if (command.empty()) {
            continue;
        }

        std::string cmd0 = command;
        cmd0.push_back('\0');

        ed247_sample_info_t info{};
        info.data = (void*)cmd0.data();
        info.size = cmd0.size();

        st = ed247_stream_push_sample(s_ctrl, &info);
        if (st != ED247_STATUS_SUCCESS) {
            std::cerr << "Failed to push sample\n";
            continue;
        }

        ed247_send_pushed_samples(ctx);

        std::cout << "Sent command via ED-247: " << command << "\n";
    }

    return 0;
}