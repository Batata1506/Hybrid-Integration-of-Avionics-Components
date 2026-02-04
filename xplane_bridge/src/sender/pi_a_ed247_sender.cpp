#include <iostream>
#include <thread>
#include <chrono>
#include <string>

#include "udp_rx.hpp"
#include "xplane_decoder.hpp"
#include "afdx_builder.hpp"

#include <ed247.h>


int main() {
    const uint16_t xplane_port  = 49000;

    UdpReceiver rx;
    if (!rx.open(xplane_port))
    {
        std::cerr << "failed to open UDP on port " << xplane_port << "\n";
        return 1;
    }

    XPlaneDataDecoder decoder;
    FlightState latest{};
    bool have_latest = false;

    ed247_context_t ctx = nullptr;

    
    ed247_status_t st = ed247_load_file("ECIC.xml", &ctx);
    if(st != ED247_STATUS_SUCCESS) {
        std::cerr << "ed247_load_file failed, status=" << st << "\n";
        return 1;
    }

    ed247_stream_t s_att = nullptr;
    ed247_stream_t s_spd = nullptr;
    ed247_stream_t s_alt = nullptr;

    if(ed247_get_stream(ctx, "VL1001_ATT", &s_att) != ED247_STATUS_SUCCESS ||
       ed247_get_stream(ctx, "VL1002_SPD", &s_spd) != ED247_STATUS_SUCCESS ||
       ed247_get_stream(ctx, "VL1003_ALT", &s_alt) != ED247_STATUS_SUCCESS) {  
        std::cerr << "ed247_get_stream failed (check names in ECIC.xml)\n";
        ed247_unload(ctx);
        return 1;
    }

    AfdxBuilder builder;

    // Each VL has its own "next time it is allowed to send" (this is BAG scheduling).
    auto next_att = std::chrono::steady_clock::now();   // VL1001_ATT (BAG 50ms)
    auto next_spd = std::chrono::steady_clock::now();   // VL1002_SPD (BAG 100ms)
    auto next_alt = std::chrono::steady_clock::now();   // VL1003_ALT (BAG 100ms)


    std::cout << "Pi A: UDP decode -> AFDX -> ED247 send\n";

    while (true)
    {
        const auto now = std::chrono::steady_clock::now();

        std::string ip;
        uint16_t src_port = 0;

        auto pkt = rx.recv_packet(&ip,&src_port);

        if(!pkt.empty()){
            auto decoded = decoder.decode(pkt);
            if (decoded.has_value())
            {
                latest = *decoded;
                have_latest = true;
            }
            
        }

        if (!have_latest)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        bool pushed_anything = false;

        // VL1001: create and push to ED247 stream
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

        // VL1002: create and push to ED247 stream
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

        // VL1003: create and push to ED247 stream
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

        // transmit queued ED247 samples
        if(pushed_anything){
            ed247_send_pushed_samples(ctx);
        }


        if (next_att < now) next_att = now;                           // Reset to now if behind
        if (next_spd < now) next_spd = now;
        if (next_alt < now) next_alt = now;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));    
    }

    ed247_unload(ctx);
    return 0;
}
