#include <iostream>             // printing
#include <cstdint>              // ints
#include <cstring>              // memcpy

#include "ed247.h"              // ED247 API
#include "afdx_pdu.hpp"         // AfdxLikeHeader + payload structs

// Read a struct from raw bytes safely
template <typename T>
static bool read_struct(const void* data, size_t len, size_t offset, T& out)
{
    if (offset + sizeof(T) > len) return false;                      // bounds check

    const uint8_t* bytes = static_cast<const uint8_t*>(data);        // convert void* to byte pointer

    std::memcpy(&out, bytes + offset, sizeof(T));                    // now arithmetic is valid
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

    ed247_internal_stream_list_t* ready_streams = nullptr;


    while (true)
    {
        // Wait up to 10 ms
        ed247_wait_during(ctx, &ready_streams, 10000);

        const void* data = nullptr;
        uint32_t size = 0;
        const ed247_timestamp_t* rts = nullptr;
        const ed247_timestamp_t* sts = nullptr;
        const ed247_sample_details_t* details = nullptr;
        bool last = false;

        // ---- Attitude ----
        while (ed247_stream_pop_sample(
                   s_att, &data, &size, &rts, &sts, &details, &last
               ) == ED247_STATUS_SUCCESS)
        {
            AFDXHeader hdr{};
            AttitudePayload pl{};

            if (read_struct(data, size, 0, hdr) &&
                read_struct(data, size, sizeof(AFDXHeader), pl))
            {
                std::cout << "[ATT VL=" << hdr.vl_id
                          << " seq=" << hdr.seq
                          << "] P=" << pl.pitch_deg
                          << " R=" << pl.roll_deg
                          << " H=" << pl.hdg_deg
                          << "\n";
            }
        }

        // ---- Airspeed ----
        while (ed247_stream_pop_sample(
                   s_spd, &data, &size, &rts, &sts, &details, &last
               ) == ED247_STATUS_SUCCESS)
        {
            AFDXHeader hdr{};
            AirSpeedPayload pl{};

            if (read_struct(data, size, 0, hdr) &&
                read_struct(data, size, sizeof(AFDXHeader), pl))
            {
                std::cout << "[SPD VL=" << hdr.vl_id
                          << " seq=" << hdr.seq
                          << "] KIAS=" << pl.kias
                          << "\n";
            }
        }

        // ---- Altitude ----
        while (ed247_stream_pop_sample(
                   s_alt, &data, &size, &rts, &sts, &details, &last
               ) == ED247_STATUS_SUCCESS)
        {
            AFDXHeader hdr{};
            AltitudePayload pl{};

            if (read_struct(data, size, 0, hdr) &&
                read_struct(data, size, sizeof(AFDXHeader), pl))
            {
                std::cout << "[ALT VL=" << hdr.vl_id
                          << " seq=" << hdr.seq
                          << "] ALT=" << pl.alt_msl_ft
                          << "\n";
            }
        }
    }

    ed247_unload(ctx);
    return 0;
}
