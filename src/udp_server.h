#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sched.h>

#include <string>

class UdpServerRaw
{

public:

    UdpServerRaw();

    // bind to local address/port
    bool bind(std::string addr, int port);

    // set receive timeout
    void set_timeout_sec(double t_sec);

    // blocking receive
    int receive(uint8_t * buffer, size_t size, bool * timeout_expired = nullptr);

    // non-blocking receive
    int try_receive(uint8_t * buffer, size_t size);

    // blocking reply to last sender (i.e., client)
    int reply(const uint8_t * buffer, size_t size);

    //
    std::string get_last_client_address() const;

private:

    int _fd;
    sockaddr_in _sv_addr;
    sockaddr_in _cl_addr;

};

template <typename rx_t, typename tx_t>
class UdpServer : protected UdpServerRaw
{

public:

    using UdpServerRaw::bind;

    using UdpServerRaw::set_timeout_sec;

    using UdpServerRaw::get_last_client_address;

    // blocking receive
    bool receive(rx_t& msg, bool * timeout_expired = nullptr)
    {
        return UdpServerRaw::receive(reinterpret_cast<uint8_t*>(&msg),
                                     sizeof(msg), timeout_expired) == sizeof(msg);
    }

    // non-blocking receive
    bool try_receive(rx_t& msg)
    {
        return UdpServerRaw::try_receive(reinterpret_cast<uint8_t*>(&msg),
                                         sizeof(msg)) == sizeof(msg);
    }

    // blocking reply to last sender (i.e., client)
    int reply(const tx_t& msg)
    {
        return UdpServerRaw::reply(reinterpret_cast<const uint8_t*>(&msg),
                                   sizeof(msg)) == sizeof(msg);
    }
};

class UdpClientRaw
{

public:

    UdpClientRaw();

    // initialize with remote server addr
    bool init(std::string addr, int port);

    // send
    int try_send(const uint8_t * buffer, size_t size);

    // non-blocking receive
    int try_receive(uint8_t * buffer, size_t size);

private:

    int _fd;
    sockaddr_in _sv_addr;

};


template <typename rx_t, typename tx_t>
class UdpClient : protected UdpClientRaw
{

public:

    // initialize with remote server addr
    using UdpClientRaw::init;

    // send
    bool try_send(const tx_t& msg)
    {
        return UdpClientRaw::try_send(reinterpret_cast<const uint8_t*>(&msg),
                                      sizeof(msg)) == sizeof(msg);
    }

    // non-blocking receive
    bool try_receive(rx_t& msg)
    {
        return UdpClientRaw::try_receive(reinterpret_cast<uint8_t*>(&msg),
                                         sizeof(msg)) == sizeof(msg);
    }

};


#endif // UDP_SERVER_H
