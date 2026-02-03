cmake_minimum_required(VERSION 3.10)
project(xplane_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Executable: Pi A sender ----
add_executable(pi_a_sender
    src/pi_a_ed247_sender.cpp
    src/udp_rx.cpp
    src/xplane_decoder.cpp
    src/afdx_builder.cpp
)

# ---- Executable: Pi B receiver ----
add_executable(pi_b_receiver
    src/pi_b_ed247_receiver.cpp
)

# ---- Include directories ----
target_include_directories(pi_a_sender PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    /usr/local/include
)

target_include_directories(pi_b_receiver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    /usr/local/include
)

# ---- Link ED247 library ----
target_link_libraries(pi_a_sender PRIVATE ed247)
target_link_libraries(pi_b_receiver PRIVATE ed247)
