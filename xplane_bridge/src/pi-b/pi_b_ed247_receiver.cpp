#include <iostream>
#include <cstdint>
#include <cstring>
#include <limits>

#include "time_utils.hpp"
#include "ed247.h"
#include "afdx_pdu.hpp"
#include "flightstate_udp_pub.hpp"
#include "flight_state.hpp"  

template <typename T>
static bool read_struct(const void* data, size_t len, size_t offset, T& out)
{
    if (offset + sizeof(T) > len) return false;
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    std::memcpy(&out, bytes + offset, sizeof(T));
    return true;
}

struct StreamStats {
    uint64_t rx_total = 0;
    uint64_t rx_1s = 0;

    bool have_last = false;
    uint32_t last_seq = 0;
    uint64_t missed = 0;

    uint64_t lat_min_us = std::numeric_limits<uint64_t>::max();
    uint64_t lat_max_us = 0;
    long double lat_sum_us = 0;
    uint64_t lat_n = 0;

    void on_rx(uint32_t seq, uint64_t tx_us, uint64_t rx_us) {
        rx_total++;
        rx_1s++;

        if (!have_last) {
            have_last = true;
            last_seq = seq;
        } else {
            uint32_t expected = last_seq + 1;
            if (seq != expected) {
                uint32_t gap = seq - expected;
                if (gap > 0 && gap < 1000000u) missed += gap;
            }
            last_seq = seq;
        }


        if (tx_us != 0 && rx_us >= tx_us) {
            uint64_t lat = rx_us - tx_us;
            if (lat < lat_min_us) lat_min_us = lat;
            if (lat > lat_max_us) lat_max_us = lat;
            lat_sum_us += (long double)lat;
            lat_n++;
        }
    }

    double lat_mean_ms() const { return lat_n ? (double)(lat_sum_us / lat_n) / 1000.0 : 0.0; }
    double lat_min_ms()  const { return lat_n ? (double)lat_min_us / 1000.0 : 0.0; }
    double lat_max_ms()  const { return lat_n ? (double)lat_max_us / 1000.0 : 0.0; }

    void reset_1s() { rx_1s = 0; }
};


int main()
{
    ed247_context_t ctx = nullptr;

    if (ed247_load_file("ECIC.xml", &ctx) != ED247_STATUS_SUCCESS) {
        std::cerr << "ed247_load_file failed\n";
        return 1;
    }

    ed247_stream_t s_att = nullptr;
    ed247_stream_t s_spd = nullptr;
    ed247_stream_t s_alt = nullptr;

    if (ed247_get_stream(ctx, "VL1001_ATT", &s_att) != ED247_STATUS_SUCCESS ||
        ed247_get_stream(ctx, "VL1002_SPD", &s_spd) != ED247_STATUS_SUCCESS ||
        ed247_get_stream(ctx, "VL1003_ALT", &s_alt) != ED247_STATUS_SUCCESS) {
        std::cerr << "ed247_get_stream failed (check ECIC.xml names)\n";
        ed247_unload(ctx);
        return 1;
    }

    std::cout << "Pi B: ED247 receive -> unpack AFDX-like -> print\n";

    ed247_internal_stream_list_t* ready_streams = nullptr;

    FlightStateUdpPublisher pub("127.0.0.1", 5555);
    FlightState latest{};

    StreamStats st_att, st_spd, st_alt;
    uint64_t last_report = now_us();


    while (true)
    {
        ed247_wait_during(ctx, &ready_streams, 10000);
        bool updated = false;

        const void* data = nullptr;
        uint32_t size = 0;
        const ed247_timestamp_t* rts = nullptr;
        const ed247_timestamp_t* sts = nullptr;
        const ed247_sample_details_t* details = nullptr;
        bool last = false;

        // ---- Attitude ----
        while (ed247_stream_pop_sample(s_att, &data, &size, &rts, &sts, &details, &last) == ED247_STATUS_SUCCESS)
        {
            AFDXHeader hdr{};
            AttitudePayload pl{};

            if (read_struct(data, size, 0, hdr) &&
                read_struct(data, size, sizeof(AFDXHeader), pl))
            {
                latest.pitch_deg = pl.pitch_deg;
                latest.roll_deg  = pl.roll_deg;
                latest.hdg_deg   = pl.hdg_deg;
                latest.seq_att = hdr.seq;
                latest.tx_time_us_att = hdr.tx_time_us;
                
                uint64_t rx_us = now_us();
                st_att.on_rx(hdr.seq, hdr.tx_time_us, rx_us);

                updated = true;

                /*std::cout << "[ATT VL=" << hdr.vl_id
                          << " seq=" << hdr.seq
                          << "] P=" << pl.pitch_deg
                          << " R=" << pl.roll_deg
                          << " H=" << pl.hdg_deg
                          << "\n";*/
            }

            
        }

        // ---- Airspeed ----
        while (ed247_stream_pop_sample(s_spd, &data, &size, &rts, &sts, &details, &last) == ED247_STATUS_SUCCESS)
        {
            AFDXHeader hdr{};
            AirSpeedPayload pl{};

            if (read_struct(data, size, 0, hdr) &&
                read_struct(data, size, sizeof(AFDXHeader), pl))
            {
                latest.kias = pl.kias;
                latest.ktas = pl.ktas;
                latest.seq_spd = hdr.seq;
                latest.tx_time_us_spd = hdr.tx_time_us;

                uint64_t rx_us = now_us();
                st_spd.on_rx(hdr.seq, hdr.tx_time_us, rx_us);

                updated = true;

                /*std::cout << "[SPD VL=" << hdr.vl_id
                          << " seq=" << hdr.seq
                          << "] KIAS=" << pl.kias
                          << " KTAS=" << pl.ktas
                          << "\n";*/
            }

            
        }

        // ---- Altitude ----
        while (ed247_stream_pop_sample(s_alt, &data, &size, &rts, &sts, &details, &last) == ED247_STATUS_SUCCESS)
        {
            AFDXHeader hdr{};
            AltitudePayload pl{};

            if (read_struct(data, size, 0, hdr) &&
                read_struct(data, size, sizeof(AFDXHeader), pl))
            {
                latest.alt_msl_ft = pl.alt_msl_ft;
                latest.seq_alt = hdr.seq;
                latest.tx_time_us_alt = hdr.tx_time_us;

                uint64_t rx_us = now_us();
                st_alt.on_rx(hdr.seq, hdr.tx_time_us, rx_us);


                updated = true;

                /*std::cout << "[ALT VL=" << hdr.vl_id
                          << " seq=" << hdr.seq
                          << "] ALT=" << pl.alt_msl_ft
                          << "\n";*/
            }
        }

        uint64_t now = now_us();
        if (now - last_report >= 1'000'000ULL) {



            auto loss_pct = [](const StreamStats& s){
                double denom = (double)(s.rx_total + s.missed);
                return denom > 0.0 ? 100.0 * (double)s.missed / denom : 0.0;
            };

            uint64_t total_1s = st_att.rx_1s + st_spd.rx_1s + st_alt.rx_1s;

            std::cout << "\n=== 1s Summary ===\n";

            std::cout << "ATT: " << st_att.rx_1s << " Hz"
                    << " | missed=" << st_att.missed << " (" << loss_pct(st_att) << "%)"
                    << " | lat(ms) mean=" << st_att.lat_mean_ms()
                    << " min=" << st_att.lat_min_ms()
                    << " max=" << st_att.lat_max_ms() << "\n";

            std::cout << "SPD: " << st_spd.rx_1s << " Hz"
                    << " | missed=" << st_spd.missed << " (" << loss_pct(st_spd) << "%)"
                    << " | lat(ms) mean=" << st_spd.lat_mean_ms()
                    << " min=" << st_spd.lat_min_ms()
                    << " max=" << st_spd.lat_max_ms() << "\n";

            std::cout << "ALT: " << st_alt.rx_1s << " Hz"
                    << " | missed=" << st_alt.missed << " (" << loss_pct(st_alt) << "%)"
                    << " | lat(ms) mean=" << st_alt.lat_mean_ms()
                    << " min=" << st_alt.lat_min_ms()
                    << " max=" << st_alt.lat_max_ms() << "\n";

            std::cout << "TOTAL: " << total_1s << " msgs/s\n";
            std::cout << "==================\n\n";

            st_att.reset_1s();
            st_spd.reset_1s();
            st_alt.reset_1s();

            // periodic (less drift than last_report = now)
            last_report += 1'000'000ULL;

            // catch up if we were delayed by >1 second
            while (now - last_report >= 1'000'000ULL) {
                last_report += 1'000'000ULL;
            }
        }


        if (updated) {
            pub.publish(latest);
        }
    }

    ed247_unload(ctx);
    return 0;
}
