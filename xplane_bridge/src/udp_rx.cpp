#include "udp_rx.hpp"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <fcntl.h>


UdpReceiver::~UdpReceiver() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}


bool UdpReceiver::open(uint16_t port) {
    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0 );

    if (sock_ < 0)
    {
        std::cerr << "socket() failed: " << std::strerror(errno) << "\n";
        return false;
    }

    int flags = ::fcntl(sock_,F_GETFL,0);
    if (flags < 0)
    {
        std::cerr << "fcntl(F_GETFL) failed: " << std::strerror(errno) << "\n";
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    if (::fcntl(sock_, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        std::cerr << "fnctl(F_SETFL) failed " << std::strerror(errno) << "\n";
        ::close(sock_);
        sock_ = -1;
        return false;
    }
    
    

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if(::bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "bind() failed: " << std::strerror(errno) << "\n";
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    return true;
    
}

std::vector<uint8_t> UdpReceiver::recv_packet(std::string* src_ip, uint16_t* src_port) {
    std::vector<uint8_t> buf(4096);

    sockaddr_in src{};

    socklen_t srclen = sizeof(src);

    const ssize_t n = ::recvfrom(
        sock_,
        buf.data(),
        buf.size(),
        0,
        reinterpret_cast<sockaddr*>(&src),
        &srclen
    );

    if (n < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)             
        {
            return {};                    
        }
        
        std::cerr << "recvfrom() failed: " <<std::strerror(errno) << "\n";
        return {};
    }

    buf.resize(static_cast<size_t>(n));

    if (src_ip)
    {
        char ip[INET_ADDRSTRLEN]{};
        ::inet_ntop(AF_INET, &src.sin_addr, ip, sizeof(ip));
        *src_ip = ip;
    }

    if (src_port) 
    {
        *src_port = ntohs(src.sin_port);
    }
    
    return buf;
    
    
}