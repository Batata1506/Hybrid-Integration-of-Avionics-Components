#pragma once

#include <cstdint>     // uint8_t, int32_t
#include <vector>      // std::vector
#include <optional>    // std::optional
#include "flight_state.hpp" // FlightState

class XPlaneDataDecoder {
public:
    std::optional<FlightState> decode(const std::vector<uint8_t>& packet) const;

private:
    // Read a 32-bit integer (4 bytes) from the packet at a byte offset.
    int32_t read_i32_le(const std::vector<uint8_t>& packet, size_t offset) const;

    // Read a 32-bit float (4 bytes) from the packet at a byte offset.
    float read_f32_le(const std::vector<uint8_t>& packet, size_t offset) const;
};
