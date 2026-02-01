#pragma once

#include <cstdint>
#include <vector>
#include <string>

class UdpReceiver {
public:
UdpReceiver() = default;

~UdpReceiver();

bool open(uint16_t port);

std::vector<uint8_t> recv_packet(std::string* src_ip = nullptr, uint16_t* src_port = nullptr);

private:
int sock_ = -1;

};
