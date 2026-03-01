#include "xplane_decoder.hpp"  // brings in the class definition

#include <cstring>   // std::memcpy (safe byte copying)
#include <iostream>  // std::cout, std::cerr


// This function reads a 32-bit integer from the packet.
// "LE" means "little-endian", which is how X-Plane sends these values on typical PCs.
int32_t XPlaneDataDecoder::read_i32_le(const std::vector<uint8_t>& packet, size_t offset) const {
    // If we don't have 4 bytes available, return 0 (safe fallback).
    if (offset + 4 > packet.size()) {
        return 0;
    }

    int32_t value = 0; // this will hold the integer result

    // Copy 4 bytes from the packet into our int variable.
    std::memcpy(&value, packet.data() + offset, 4);

    // Return the integer.
    return value;
}

// This function reads a 32-bit float from the packet.
float XPlaneDataDecoder::read_f32_le(const std::vector<uint8_t>& packet, size_t offset) const {
    // If we don't have 4 bytes available, return 0.0 (safe fallback).
    if (offset + 4 > packet.size()) {
        return 0.0f;
    }

    float value = 0.0f; // this will hold the float result

    // Copy 4 bytes from the packet into our float variable.
    std::memcpy(&value, packet.data() + offset, 4);

    // Return the float.
    return value;
}

std::optional<FlightState> XPlaneDataDecoder::decode(const std::vector<uint8_t>& packet) const {
    // DATA packets must be at least 5 bytes: 'D' 'A' 'T' 'A' '\0'
    if (packet.size() < 5) {
        return std::nullopt;
    }

    // Read the first 4 bytes as text so we can check the tag.
    char tag[5] = {};                               // 4 chars + null terminator
    tag[0] = static_cast<char>(packet[0]);          // 'D'
    tag[1] = static_cast<char>(packet[1]);          // 'A'
    tag[2] = static_cast<char>(packet[2]);          // 'T'
    tag[3] = static_cast<char>(packet[3]);          // 'A'

    // If it's not a DATA packet, we refuse to decode it.
    if (std::strcmp(tag, "DATA") != 0) {
        return std::nullopt;
    }

    FlightState state; 

    // After "DATA\0", the blocks begin at byte offset 5.
    size_t offset = 5;

    // Each block is 36 bytes:
    // - 4 bytes: int group index
    // - 32 bytes: 8 floats (8 * 4 bytes)
    while (offset + 36 <= packet.size()) {
        // Read group index as an int.
        int32_t group = read_i32_le(packet, offset);

        // Print the group number so we can see what X-Plane is sending.
        //std::cout << "DATA group index: " << group << "\n";

        float f[8];
        for (int i = 0; i < 8; i++)
        {
            f[i] = read_f32_le(packet, offset + 4 + (i*4));
        }

       


        if (group == 0)
        {
            state.fps_actual = f[0];
            state.fps_sim = f[1];
        }
        

        else if (group == 3) {
            state.kias = f[0];
            state.ktas = f[2];
            state.ktgs = f[3];

        }

        else if (group == 17){
            state.pitch_deg = f[0];
            state.roll_deg = f[1];
            state.hdg_deg = f[2];

        }

        else if (group == 20)
        {
            state.alt_msl_ft = f[2];
        }
        
        

        // Move to the next block.
        offset += 36;
    }

    return state;
}
