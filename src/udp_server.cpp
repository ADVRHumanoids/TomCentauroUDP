#include "udp_server.h"
#include <math.h>

UdpServerRaw::UdpServerRaw()
{

}

bool UdpServerRaw::bind(std::string addr, int port)
{
    // create socket file descriptor
    _fd = socket(AF_INET, SOCK_DGRAM, 0);

    if(_fd < 0)
    {
        perror("socket creation failed");
        return false;
    }

    // initialize addr structs
    memset(&_sv_addr, 0, sizeof(_sv_addr));
    memset(&_cl_addr, 0, sizeof(_cl_addr));

    // fill server information
    _sv_addr.sin_family = AF_INET;
    _sv_addr.sin_port = htons(port);

    if(inet_aton(addr.c_str(), &_sv_addr.sin_addr) == 0)
    {
        fprintf(stderr, "invalid address '%s'\n", addr.c_str());
        return false;
    }

    // bind
    if(::bind(_fd, (const struct sockaddr *)&_sv_addr, sizeof(_sv_addr)) < 0)
    {
        perror("bind failed");
        return false;
    }

    return true;
}

void UdpServerRaw::set_timeout_sec(double t_sec)
{
    struct timeval tv;
    tv.tv_sec = floor(t_sec);
    tv.tv_usec = (t_sec - tv.tv_sec)*1e6;
    if (setsockopt(_fd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0)
    {
        perror("set_timeout_sec failed");
    }
}

int UdpServerRaw::receive(uint8_t *buffer, size_t size, bool * timeout_expired)
{
    socklen_t cl_addr_size = sizeof(_cl_addr);

    int ret = ::recvfrom(_fd,
                         buffer,
                         size,
                         MSG_WAITALL,
                         (struct sockaddr *)&_cl_addr,
                         &cl_addr_size);

    if(ret == -1 &&
        errno != EAGAIN && errno != EWOULDBLOCK)
    {
        perror("recvfrom failed");
    }

    if(timeout_expired)
    {
        *timeout_expired = errno == EAGAIN || errno == EWOULDBLOCK;
    }

    return ret;
}

int UdpServerRaw::try_receive(uint8_t *buffer, size_t size)
{
    socklen_t cl_addr_size = sizeof(_cl_addr);

    int ret = ::recvfrom(_fd,
                         buffer,
                         size,
                         MSG_DONTWAIT,
                         (struct sockaddr *)&_cl_addr,
                         &cl_addr_size);

    if(ret == -1 &&
            errno != EAGAIN && errno != EWOULDBLOCK)
    {
        perror("recvfrom failed");
    }

    return ret;
}

int UdpServerRaw::reply(const uint8_t *buffer, size_t size)
{
    socklen_t cl_addr_size = sizeof(_cl_addr);

    int ret = ::sendto(_fd,
                       buffer,
                       size,
                       MSG_CONFIRM,
                       (const struct sockaddr *)&_cl_addr,
                       cl_addr_size);

    if(ret == -1)
    {
        perror("sendto failed");
    }

    return ret;
}

std::string UdpServerRaw::get_last_client_address() const
{
    std::string addr = inet_ntoa(_cl_addr.sin_addr);
    return addr + ":" + std::to_string(ntohs(_cl_addr.sin_port));
}

UdpClientRaw::UdpClientRaw()
{

}

bool UdpClientRaw::init(std::string addr, int port)
{
    // create socket file descriptor
    _fd = socket(AF_INET, SOCK_DGRAM, 0);

    if(_fd < 0)
    {
        perror("socket creation failed");
        return false;
    }

    // initialize addr structs
    memset(&_sv_addr, 0, sizeof(_sv_addr));

    // fill server information
    _sv_addr.sin_family = AF_INET;
    _sv_addr.sin_port = htons(port);

    if(inet_aton(addr.c_str(), &_sv_addr.sin_addr) == 0)
    {
        fprintf(stderr, "invalid address '%s'\n", addr.c_str());
        return false;
    }

    return true;
}

int UdpClientRaw::try_send(const uint8_t *buffer, size_t size)
{
    socklen_t sv_addr_size = sizeof(_sv_addr);

    int ret = ::sendto(_fd,
                       buffer,
                       size,
                       MSG_DONTWAIT,
                       (const struct sockaddr *)&_sv_addr,
                       sv_addr_size);

    if(ret == -1 &&
            (errno != EAGAIN || errno != EWOULDBLOCK))
    {
        perror("sendto failed");
    }

    return ret;
}

int UdpClientRaw::try_receive(uint8_t *buffer, size_t size)
{
    socklen_t sv_addr_size = sizeof(_sv_addr);

    int ret = ::recvfrom(_fd,
                         buffer,
                         size,
                         MSG_DONTWAIT,
                         (struct sockaddr *)&_sv_addr,
                         &sv_addr_size);

    if(ret == -1 &&
            (errno != EAGAIN || errno != EWOULDBLOCK))
    {
        perror("recvfrom failed");
    }

    return ret;
}
