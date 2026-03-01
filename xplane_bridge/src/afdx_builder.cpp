#include "afdx_builder.hpp"
#include "time_utils.hpp"   // now_us()

AfdxBuilder::AfdxBuilder(){

}

std::vector<uint8_t> AfdxBuilder::build_attitude(const FlightState& st) {
    std::vector<uint8_t> out;

    // Create payload
    AttitudePayload payload{};
    payload.pitch_deg   = st.pitch_deg;
    payload.roll_deg    = st.roll_deg;
    payload.hdg_deg = st.hdg_deg;

    // Create header
    AFDXHeader hdr{};
    hdr.version     = 1;
    hdr.msg_type    = 1;         // 1 = attitude
    hdr.vl_id       = 1001;      // VL1001
    hdr.seq         = ++seq_att_;
    hdr.tx_time_us  = now_us();
    hdr.payload_len = sizeof(payload);
    hdr.flags       = 0;         // can use bits if want to implement channel A/B

    // Convert header + payload into bytes
    append_struct(out, hdr);
    append_struct(out, payload);

    return out;
}

std::vector<uint8_t> AfdxBuilder::build_airspeed(const FlightState& st){
    std::vector<uint8_t> out;

    AirSpeedPayload payload{};
    payload.kias = st.kias;
    payload.ktas = st.ktas;

    AFDXHeader hdr{};
    hdr.version = 1;
    hdr.msg_type = 2;
    hdr.vl_id = 1002;
    hdr.seq = ++seq_spd_;
    hdr.tx_time_us = now_us();
    hdr.payload_len = sizeof(payload);
    hdr.flags = 0;

    append_struct(out, hdr);
    append_struct(out, payload);

    return out;
}

std::vector<uint8_t> AfdxBuilder::build_altitude(const FlightState& st){
    std::vector<uint8_t> out;

    AltitudePayload payload{};
    payload.alt_msl_ft = st.alt_msl_ft;

    AFDXHeader hdr{};
    hdr.version = 1;
    hdr.msg_type = 3;
    hdr.vl_id = 1003;
    hdr.seq = ++seq_alt_;
    hdr.tx_time_us = now_us();
    hdr.payload_len = sizeof(payload);
    hdr.flags = 0;

    append_struct(out, hdr);
    append_struct(out, payload);

    return out;
}