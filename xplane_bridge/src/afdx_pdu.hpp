#pragma once

#include <cstdint>

#pragma pack(push, 1)

struct AFDXHeader
{
     uint16_t version;      // Version of our format (start at 1)
    uint16_t msg_type;     // What payload this is (1=attitude, 2=airspeed, 3=altitude)
    uint32_t vl_id;        // Virtual Link ID (e.g., 1001, 1002, 1003)
    uint32_t seq;          // Sequence counter (increments per VL)
    uint64_t tx_time_us;   // Timestamp (microseconds) when sender created message
    uint16_t payload_len;  // Number of bytes that follow in the payload
    uint16_t flags;        // Bit flags (optional): e.g., channel A/B, validity bits
};

struct AttitudePayload
{
    float pitch_deg; // from group 17 all three
    float roll_deg;
    float hdg_deg;
};

struct AirSpeedPayload
{
    float kias; // from group 3
};


struct AltitudePayload
{
    float alt_msl_ft; // from group 20
};

#pragma pack(pop)

