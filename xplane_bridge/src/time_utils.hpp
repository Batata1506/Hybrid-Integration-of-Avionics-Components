#pragma once

#include <cstdint>
#include <chrono> // time utils

inline uint64_t now_us(){
    using namespace std::chrono;

    auto now = system_clock::now();
    auto us = duration_cast<microseconds>(now.time_since_epoch());

    return static_cast<uint64_t>(us.count());
}
