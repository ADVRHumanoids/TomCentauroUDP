#ifndef SOCKETBASE_H
#define SOCKETBASE_H

#pragma once

#include <memory>
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
// Winsock Library
#pragma comment(lib, "ws2_32.lib")
#elif __linux__

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
// Non blocking socket
#include <fcntl.h>
#include <memory>
#define SOCKET int
#define SOCKET_ERROR (SOCKET)(-1)
#define NO_ERROR 0

#endif
/**
 * @brief Initialize socket object with data structure of transmission message
 * and receiver message as template parameter TX, RX respectively.
 *
 * @tparam TX
 * @tparam RX
 */
template <typename TX, typename RX = TX>
class SocketBase
{
public:
    virtual ~SocketBase<TX, RX>();

    virtual void initialize() = 0;

    /**
     * @brief Bind socket to local address and port
     *
     * @param address IP address string of local node
     * @param port port of local node
     */
    void bindLocal(const char *address, uint16_t port);

    /**
     * @brief Connect to remote address and port
     * 
     * @param address IP address string of remote node
     * @param port port of remote node
     */
    void connectRemote(const char *address, uint16_t port);

    /**
     * @brief Set the Receive Buffer Size
     *
     * @param tx_buff
     */
    void setReceiveBufferSize(size_t tx_buff);

    /**
     * @brief Send data to socket
     *
     * @param tx
     * @param size
     * @return int
     */
    int socketSend(TX &tx, const size_t size = 0);

    /**
     * @brief Receive data from socket
     *
     * @param rx
     * @param size
     * @return int
     */
    int socketReceive(RX &rx, size_t size = 0);

    typedef std::unique_ptr<SocketBase> ptr;

protected: // protected constructor so object can not be instanciated from base class
    /**
     * @brief Construct a new Socket Base< TX,  RX> object
     *
     */
    SocketBase<TX, RX>();

    /**
     * @brief holds byte array to be sent
     */
    std::unique_ptr<char[]> _tx_buffer;

    /**
     * @brief holds received byte array
     */
    std::unique_ptr<char[]> _rx_buffer;

    size_t _tx_size;

    size_t _rx_size;

#ifdef _WIN32
    WSADATA wsaData;
#endif
    SOCKET _socket_descriptor;

    struct sockaddr_in _local_socket;

    struct sockaddr_in _remote_socket;
};

// definations

template <typename TX, typename RX>
SocketBase<TX, RX>::SocketBase()
{
    _rx_size = sizeof(RX);
    _rx_buffer = std::make_unique<char[]>(_rx_size);
    _tx_size = sizeof(TX);
    _tx_buffer = std::make_unique<char[]>(_tx_size);
}

template <typename TX, typename RX>
SocketBase<TX, RX>::~SocketBase()
{
#ifdef _WIN32
    closesocket(_socket_descriptor);
#elif __linux__
    close(_socket_descriptor);
#endif
}

template <typename TX, typename RX>
void SocketBase<TX, RX>::bindLocal(const char *address, uint16_t port)
{
    /* zero out structure */
    memset((char *)(&_local_socket), 0, sizeof(_local_socket));

    /* initialize address to bind */
    _local_socket.sin_family = AF_INET;
    _local_socket.sin_port = htons(port);
    _local_socket.sin_addr.s_addr = inet_addr(address);
    bzero(&(_local_socket.sin_zero), 8); /* zero the rest of the struct */

    /* bind socket to address and port */
    if (bind(_socket_descriptor, (struct sockaddr *)(&_local_socket), sizeof(_local_socket)) != NO_ERROR)
    {
#ifdef _WIN32
        closesocket(_socket_descriptor);
#elif __linux__
        close(_socket_descriptor);
#endif
        throw std::runtime_error("bind() failed");
    }
}

template <typename TX, typename RX>
void SocketBase<TX, RX>::connectRemote(const char *address, uint16_t port)
{
    memset((char *)&_remote_socket, 0, sizeof(_remote_socket));
    _remote_socket.sin_family = AF_INET;
    _remote_socket.sin_port = htons(port);
    _remote_socket.sin_addr.s_addr = inet_addr(address);

    if (connect(_socket_descriptor, (struct sockaddr *)(&_remote_socket), sizeof(_remote_socket)) < 0)
    {
#ifdef _WIN32
        closesocket(_socket_descriptor);
#elif __linux__
        close(_socket_descriptor);
#endif
        throw std::runtime_error("connect() failed");
    }
}

template <typename TX, typename RX>
int SocketBase<TX, RX>::socketSend(TX &tx, const size_t size)
{
    if (size)
    {
        _tx_size = size;
    }

    memcpy(_tx_buffer.get(), &tx, _tx_size);

    int bytes = send(_socket_descriptor, // Connected socket
                     _tx_buffer.get(),   // Data buffer
                     _tx_size,           // Length of data
                     0);

    return bytes;
}

template <typename TX, typename RX>
int SocketBase<TX, RX>::socketReceive(RX &rx, size_t size)
{
    if (size)
    {
        _rx_size = size;
    }

    /* The non blocking area */
    fcntl(_socket_descriptor, F_SETFL, O_NONBLOCK);
    ssize_t bytes = recv(_socket_descriptor, // Bound socket
                         _rx_buffer.get(),   // Data buffer
                         _rx_size,           // Length of data
                         0);

    if (bytes > 0)
    {
        memcpy(&rx, _rx_buffer.get(), _rx_size);
        
    }

    return bytes;
}

template <typename TX, typename RX>
void SocketBase<TX, RX>::setReceiveBufferSize(size_t sendBuff)
{
    socklen_t optlen = sizeof(int);
    size_t sendbuff;

    if (setsockopt(
            this->_socket_descriptor, SOL_SOCKET, SO_RCVBUF, &sendbuff, sizeof(sendbuff)) == SOCKET_ERROR)
    {
        throw std::runtime_error("error setsockopt()");
    }

    // Get buffer size
    auto res = getsockopt(this->_socket_descriptor, SOL_SOCKET, SO_RCVBUF, &sendbuff, &optlen);

    if (res == -1)
    {
        throw std::runtime_error("error getsockopt()");
    }
    else
    {
        printf("revc buffer size = %d\n", sendbuff);
    }
}



#endif
