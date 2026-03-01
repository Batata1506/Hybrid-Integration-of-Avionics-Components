#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "udp_rx.hpp"
#include "xplane_decoder.hpp"
#include "afdx_builder.hpp"

#include <ed247.h>

// Send an X-Plane UDP CMND packet: "CMND\0" + <command> + "\0"
static bool send_xplane_cmnd(int sock, const sockaddr_in& xplane_addr, const std::string& cmd)
{
    std::string payload;
    payload.reserve(5 + cmd.size() + 1);
    payload.append("CMND", 4);
    payload.push_back('\0');
    payload.append(cmd);
    payload.push_back('\0');

    const ssize_t n = ::sendto(sock,
                              payload.data(),
                              payload.size(),
                              0,
                              reinterpret_cast<const sockaddr*>(&xplane_addr),
                              sizeof(xplane_addr));
    return n == (ssize_t)payload.size();
}

int main() {
    const uint16_t xplane_port  = 49000;

    // UDP receiver for X-Plane DATA packets (incoming)
    UdpReceiver rx;
    if (!rx.open(xplane_port))
    {
        std::cerr << "failed to open UDP on port " << xplane_port << "\n";
        return 1;
    }

    XPlaneDataDecoder decoder;
    FlightState latest{};
    bool have_latest = false;

    // ED-247
    ed247_context_t ctx = nullptr;
    ed247_status_t st = ed247_load_file("ECIC.xml", &ctx);
    if(st != ED247_STATUS_SUCCESS) {
        std::cerr << "ed247_load_file failed, status=" << st << "\n";
        return 1;
    }

    ed247_stream_t s_att = nullptr;
    ed247_stream_t s_spd = nullptr;
    ed247_stream_t s_alt = nullptr;
    ed247_stream_t s_ctrl = nullptr; // NEW: control stream (PiB->PiA)

    if(ed247_get_stream(ctx, "VL1001_ATT", &s_att) != ED247_STATUS_SUCCESS ||
       ed247_get_stream(ctx, "VL1002_SPD", &s_spd) != ED247_STATUS_SUCCESS ||
       ed247_get_stream(ctx, "VL1003_ALT", &s_alt) != ED247_STATUS_SUCCESS ||
       ed247_get_stream(ctx, "VL2001_CTRL", &s_ctrl) != ED247_STATUS_SUCCESS) {
        std::cerr << "ed247_get_stream failed (check names in ECIC.xml)\n";
        ed247_unload(ctx);
        return 1;
    }

    // UDP sender socket for X-Plane CMND packets (outgoing)
    int xplane_sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (xplane_sock < 0) {
        std::cerr << "socket() for X-Plane CMND failed\n";
        ed247_unload(ctx);
        return 1;
    }

    bool have_xplane_addr = false;
    sockaddr_in xplane_addr{};
    xplane_addr.sin_family = AF_INET;
    xplane_addr.sin_port = htons(xplane_port);

    AfdxBuilder builder;

    auto next_att = std::chrono::steady_clock::now();   // VL1001_ATT (BAG 50ms)
    auto next_spd = std::chrono::steady_clock::now();   // VL1002_SPD (BAG 100ms)
    auto next_alt = std::chrono::steady_clock::now();   // VL1003_ALT (BAG 100ms)

    std::cout << "Pi A: UDP decode -> AFDX -> ED247 send\n";
    std::cout << "Pi A: ED247 CTRL receive -> X-Plane CMND send\n";

    // Used for non-blocking ED-247 receive
    ed247_internal_stream_list_t* ready_streams = nullptr;

    while (true)
    {
        const auto now = std::chrono::steady_clock::now();

        // ---------------------------------------------------------
        // (1) Receive X-Plane DATA (non-blocking) and decode state
        // ---------------------------------------------------------
        std::string ip;
        uint16_t src_port = 0;
        auto pkt = rx.recv_packet(&ip, &src_port);

        if(!pkt.empty()){
            // Learn X-Plane IP from incoming packets (so no hardcoding)
            if (!have_xplane_addr) {
                if (::inet_pton(AF_INET, ip.c_str(), &xplane_addr.sin_addr) == 1) {
                    have_xplane_addr = true;
                    std::cout << "[XPLANE] Detected IP: " << ip << ":" << xplane_port << "\n";
                }
            }

            auto decoded = decoder.decode(pkt);
            if (decoded.has_value())
            {
                latest = *decoded;
                have_latest = true;
            }
        }

        // ---------------------------------------------------------
        // (2) Receive ED-247 (non-blocking) so we can pop CTRL samples
        // ---------------------------------------------------------
        // 0us timeout = poll only (don’t block)
        ed247_wait_during(ctx, &ready_streams, 0);

        // ---------------------------------------------------------
        // (3) Pop control samples and forward to X-Plane (CMND)
        // ---------------------------------------------------------
        {
            const void* data = nullptr;
            uint32_t size = 0;
            const ed247_timestamp_t* rts = nullptr;
            const ed247_timestamp_t* sts = nullptr;
            const ed247_sample_details_t* details = nullptr;
            bool last = false;

            while (ed247_stream_pop_sample(s_ctrl, &data, &size, &rts, &sts, &details, &last) == ED247_STATUS_SUCCESS)
            {
                if (!data || size == 0) continue;

                // We expect a null-terminated command string
                const char* cstr = static_cast<const char*>(data);
                std::string cmd(cstr);

                if (!cmd.empty()) {
                    if (!have_xplane_addr) {
                        std::cerr << "[CTRL] RX: " << cmd << " (ignored: X-Plane IP not detected yet)\n";
                    } else {
                        bool ok = send_xplane_cmnd(xplane_sock, xplane_addr, cmd);
                        std::cout << "[CTRL] RX: " << cmd << " -> X-Plane " << (ok ? "OK" : "FAIL") << "\n";
                    }
                }
            }
        }

        // ---------------------------------------------------------
        // (4) Send telemetry streams (only if we have decoded state)
        // ---------------------------------------------------------
        if (!have_latest)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        bool pushed_anything = false;

        if (now >= next_att)
        {
            auto att = builder.build_attitude(latest);
            ed247_stream_push_sample(
                s_att,
                att.data(),
                (uint32_t)att.size(),
                nullptr,
                nullptr
            );

            next_att += std::chrono::milliseconds(50);
            pushed_anything = true;
        }

        if (now >= next_spd)
        {
            auto spd = builder.build_airspeed(latest);
            ed247_stream_push_sample(
                s_spd,
                spd.data(),
                (uint32_t)spd.size(),
                nullptr,
                nullptr
            );

            next_spd += std::chrono::milliseconds(100);
            pushed_anything = true;
        }

        if (now >= next_alt)
        {
            auto alt = builder.build_altitude(latest);
            ed247_stream_push_sample(
                s_alt,
                alt.data(),
                (uint32_t)alt.size(),
                nullptr,
                nullptr
            );

            next_alt += std::chrono::milliseconds(100);
            pushed_anything = true;
        }

        if(pushed_anything){
            ed247_send_pushed_samples(ctx);
        }

        // Reset schedules if behind
        if (next_att < now) next_att = now;
        if (next_spd < now) next_spd = now;
        if (next_alt < now) next_alt = now;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Unreachable in this loop, but kept for completeness
    ::close(xplane_sock);
    ed247_unload(ctx);
    return 0;
}