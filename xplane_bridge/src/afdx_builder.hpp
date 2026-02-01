#pragma once

#include <cstdint>
#include <vector>

#include "flight_state.hpp"
#include "afdx_pdu.hpp"

class AfdxBuilder{

    public:
        AfdxBuilder();

        // Build VL1001 message
        std::vector<uint8_t> build_attitude(const FlightState& st);
        // Build VL1002 message
        std::vector<uint8_t> build_airspeed(const FlightState& st);
        // Build VL1003 message
        std::vector<uint8_t> build_altitude(const FlightState& st);

    private:
        // Sequence counters for each VL (AFDX Concept)
        uint32_t seq_att_ = 0;
        uint32_t seq_spd_ = 0;
        uint32_t seq_alt_ = 0;

        template <typename T>
            void append_struct(std::vector<uint8_t>& out, const T& s){
                const uint8_t* p = reinterpret_cast<const uint8_t*>(&s);
                out.insert(out.end(),p,p+sizeof(T));
            }

};