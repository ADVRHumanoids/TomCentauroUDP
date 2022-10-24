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

class UdpServer
{

public:

    UdpServer();

    // bind to local address/port
    bool bind(std::string addr, int port);

    // blocking receive
    int receive(uint8_t * buffer, size_t size);

    // non-blocking receive
    int try_receive(uint8_t * buffer, size_t size);

    // blocking reply to last sender (i.e., client)
    int reply(const uint8_t * buffer, size_t size);

private:

    int _fd;
    sockaddr_in _sv_addr;
    sockaddr_in _cl_addr;

};

class UdpClient
{

public:

    UdpClient();

    // initialize with remote server addr
    bool init(std::string addr, int port);

    // send
    int send(uint8_t * buffer, size_t size);

    // non-blocking receive
    int try_receive(uint8_t * buffer, size_t size);

private:

    int _fd;
    sockaddr_in _sv_addr;

};

#endif // UDP_SERVER_H
