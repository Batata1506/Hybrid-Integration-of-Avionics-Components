#include <iostream>             // printing
#include <cstdint>              // ints
#include <cstring>              // memcpy

#include "ed247.h"              // ED247 API
#include "afdx_pdu.hpp"         // AfdxLikeHeader + payload structs

// Read a struct from raw bytes safely
template <typename T>
static bool read_struct(const uint8_t* data, size_t len, size_t offset, T& out)
{
    if (offset + sizeof(T) > len) return false;           // bounds check
    std::memcpy(&out, data + offset, sizeof(T));          // copy bytes into struct
    return true;
}

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

    while (true)
    {
        // Wait up to 10 ms for new data (prevents 100% CPU spinning)
        ed247_wait_during(ctx, 10000);

        // Buffer for incoming sample
        uint8_t buf[512];
        uint32_t size = sizeof(buf);

        // ---- Pop attitude samples ----
        while (ed247_stream_pop_sample(s_att, buf, &size) == ED247_STATUS_SUCCESS) {
            AFDXHeader hdr{};
            if (!read_struct(buf, size, 0, hdr)) break;

            AttitudePayload pl{};
            if (!read_struct(buf, size, sizeof(AFDXHeader), pl)) break;

            std::cout << "[VL" << hdr.vl_id << " seq=" << hdr.seq << "] "
                      << "PITCH=" << pl.pitch_deg
                      << " ROLL=" << pl.roll_deg
                      << " HDG=" << pl.hdg_deg
                      << "\n";

            size = sizeof(buf); // reset size for next pop
        }

        // ---- Pop airspeed samples ----
        size = sizeof(buf);
        while (ed247_stream_pop_sample(s_spd, buf, &size) == ED247_STATUS_SUCCESS) {
            AFDXHeader hdr{};
            if (!read_struct(buf, size, 0, hdr)) break;

            AirSpeedPayload pl{};
            if (!read_struct(buf, size, sizeof(AFDXHeader), pl)) break;

            std::cout << "[VL" << hdr.vl_id << " seq=" << hdr.seq << "] "
                      << "KIAS=" << pl.kias
                      << "\n";

            size = sizeof(buf);
        }

        // ---- Pop altitude samples ----
        size = sizeof(buf);
        while (ed247_stream_pop_sample(s_alt, buf, &size) == ED247_STATUS_SUCCESS) {
            AFDXHeader hdr{};
            if (!read_struct(buf, size, 0, hdr)) break;

            AltitudePayload pl{};
            if (!read_struct(buf, size, sizeof(AFDXHeader), pl)) break;

            std::cout << "[VL" << hdr.vl_id << " seq=" << hdr.seq << "] "
                      << "ALT(ft)=" << pl.alt_msl_ft
                      << "\n";

            size = sizeof(buf);
        }
    }

    ed247_unload(ctx);
    return 0;
}
