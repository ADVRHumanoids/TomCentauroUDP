#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#pragma once

#include "SocketBase.h"

template <typename TX, typename RX = TX>
class UDPSocket : public SocketBase<TX, RX>
{
public:
    UDPSocket() = default;
    /**
     * @brief Initialize UDP socket connection
     * 
     */
    void initialize() override;
    /**
     * @brief Set the UDP Receive Buffer Size object
     * 
     * @param rx_buffer 
     */
    void setReceiveBufferSize(size_t &rx_buffer);
};

// definations

template <typename TX, typename RX>
void UDPSocket<TX, RX>::initialize()
{

#ifdef _WIN32
    /* Initialize Winsock */
    // consoleLog->info("Initialising Winsock...");
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        WSACleanup();
        // init failed;
    }
#endif

    // create socket
    if ((this->_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
    {
        perror("socket() udp");
        throw std::runtime_error("socket() UDP");
    }

#ifdef __linux__
    int option = 1;
    /* Non-blocking */
    ioctl(this->_socket_descriptor, FIONBIO, &option);

    // Non-blocking//
    // int flags = fcntl(this->_socket_descriptor, F_GETFL, 0);
    // fcntl(this->_socket_descriptor, F_SETFL, flags | O_NONBLOCK);
#endif
}

template <typename TX, typename RX>
void UDPSocket<TX, RX>::setReceiveBufferSize(size_t &tx_buffer)
{
    socklen_t optlen = sizeof(int);
    //size_t tx_buffer;

    if (setsockopt(this->_socket_descriptor, SOL_SOCKET, SO_RCVBUF, &tx_buffer, sizeof(tx_buffer)) == SOCKET_ERROR)
    {
        throw std::runtime_error("error setsockopt()");
    }

    // Get buffer size
    auto res = getsockopt(this->_socket_descriptor, SOL_SOCKET, SO_RCVBUF, &tx_buffer, &optlen);

    if (res == -1)
    {
        throw std::runtime_error("error getsockopt()");
    }
    else
    {
        printf("revc buffer size = %d\n", tx_buffer);
    }
}


#endif