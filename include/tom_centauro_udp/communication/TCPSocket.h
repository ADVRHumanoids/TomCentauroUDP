#ifndef TCPSOCKET_H
#define TCPSOCKET_H

#pragma once

#include "SocketBase.h"

template <typename TX, typename RX = TX>
class TCPSocket : public SocketBase<TX, RX>
{
public:
    TCPSocket() = default;

    /**
     * @brief initialize TCP socket connection
     *
     */
    void initialize() override;
};

// definations


template <typename TX, typename RX>
void TCPSocket<TX, RX>::initialize()
{
    if ((this->_socket_descriptor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == SOCKET_ERROR)
    {
        throw std::runtime_error("Could not create TCP socket");
    }
}



#endif