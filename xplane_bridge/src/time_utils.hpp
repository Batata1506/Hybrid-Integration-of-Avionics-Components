#pragma once

#include <cstdint>
#include <chrono> // time utils

inline uint64_t now_us(){
    // Get current time from the system clock
    auto now = std::chrono::system_clock::now();
    // convert to microseconds
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    // Return as a simple int
    return static_cast<uint64_t>(us.count());

}