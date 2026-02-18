#pragma once
#include <cstdint>

struct FlightState {
    
    float fps_actual  = 0.0f;
    float fps_sim = 0.0f;

    float kias = 0.0f;
    float ktas = 0.0f;
    float ktgs = 0.0f;

    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    float hdg_deg = 0.0f;

    float alt_msl_ft = 0.0f;

    uint32_t seq_att = 0;
    uint32_t seq_spd = 0;
    uint32_t seq_alt = 0;

    uint64_t tx_time_us_att = 0;
    uint64_t tx_time_us_spd = 0;
    uint64_t tx_time_us_alt = 0;
};
