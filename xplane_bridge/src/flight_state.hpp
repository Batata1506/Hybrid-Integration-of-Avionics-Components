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
};
