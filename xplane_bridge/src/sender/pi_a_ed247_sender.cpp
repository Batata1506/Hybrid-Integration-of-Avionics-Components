#include <iostream>
#include <thread>
#include <chrono>
#include <string>


#include "udp_rx.hpp"
#include "xplane_decoder.hpp"
#include "afdx_builder.hpp"

#include <ed247.h>

static void sleep_until_next(std::chrono::steady_clock::time_point& next, int period_ms){
    next += std::chrono::milliseconds(period_ms);
    std::this_thread::sleep_until(next);
}

int main() {
    const uint16_t xplane_port  = 49000;

    UdpReceiver rx;
    if (!rx.open(xplane_port))
    {
        std::cerr << "failed to Open UDP on Port " << xplane_port << "\n";
        return 1;
    }

    XPlaneDataDecoder decoder;
    FlightState latest{};
    bool have_latest = false;

    ed247_context_t ctx = nullptr;
    if(ed247_load_file("ECIC.xml", &ctx) != ED247_STATUS_SUCCESS) {
        std::cerr << "ed247_load_file failed (ECIC.xml not found or invalid)\n";
        return 1;
    }

    ed247_stream_t s_att = nullptr;
    ed247_stream_t s_spd = nullptr;
    ed247_stream_t s_alt = nullptr;

    if(ed247_get_stream(ctx, "VL1001_ATT", &s_att) != ED247_STATUS_SUCCESS ||
       ed247_get_stream(ctx, "VL1002_SPD", &s_spd) != ED247_STATUS_SUCCESS ||
       ed247_get_strem(ctx, "VL1003_ALT", &s_alt) != ED247_STATUS_SUCCESS){
        std::cerr << "ed247_get_stream failed (check names in ECIC.xml)\n";
        return 1;
       }

       ed247_stream_assistant_t a_att = nullptr;
       ed247_stream_assistant_t a_spd = nullptr;
       ed247_stream_assistant_t a_alt = nullptr;

       if (ed247_stream_get_assistant(s_att, &a_att) != ED247_STATUS_SUCCESS ||
            ed247_stream_get_assistant(s_spd, &a_spd) != ED247_STATUS_SUCCESS ||
            ed247_stream_get_assistant(s_alt, &a_alt) != ED247_STATUS_SUCCESS)
       {
        std::cerr << "ed247_stream_get_assistant failed\n";
        return 1;
       }

       AfdxBuilder builder;

       auto next_20hz = std::chrono::steady_clock::now();
       auto next_10hz = std::chrono::steady_clock::now();

       std::cout << "Pi A: UDP decode -> AFDX -> ED247 send\n";

       while (true)
       {
        std::string ip;
        uint16_t src_port = 0;
        auto pkt = rx.recv_packet(&ip,&src_port);

        if(!pkt.empty()){
            auto st = decoder.decode(pkt);
            if (st.has_value())
            {
                latest = *st;
                have_latest = true;
            }
            
        }

        if (!have_latest)
        {
            continue;
        }

        if (std::chrono::steady_clock::now() >= next_20hz)
        {
            auto att = builder.build_attitude(latest);

            ed247_stream_push_sample(s_att,att.data(), att.size());

            ed247_stream_assistants_written_push_samples(ctx);

            sleep_until_next(next_20hz, 50);

        }

        if (std::chrono::steady_clock::now() >= next_10hz)
        {
            auto spd = builder.build_airspeed(latest);
            auto alt = builder.build_altitude(latest);

            ed247_stream_push_sample(s_spd,spd.data(), spd.size());
            ed247_stream_push_sample(s_alt,alt.data(), alt.size());

            ed247_stream_assistants_written_push_samples(ctx);

            sleep_until_next(next_10hz, 100);
        }
        
       }

       ed247_unload(ctx);
        return 0;
       
}

