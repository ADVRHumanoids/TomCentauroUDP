#include "udp_server.h"

UdpServer::UdpServer()
{

}

bool UdpServer::bind(std::string addr, int port)
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

int UdpServer::receive(uint8_t *buffer, size_t size)
{
    socklen_t cl_addr_size = sizeof(_cl_addr);

    int ret = ::recvfrom(_fd,
                         buffer,
                         size,
                         MSG_WAITALL,
                         (struct sockaddr *)&_cl_addr,
                         &cl_addr_size);

    if(ret == -1)
    {
        perror("recvfrom failed");
    }

    return ret;
}

int UdpServer::try_receive(uint8_t *buffer, size_t size)
{
    socklen_t cl_addr_size = sizeof(_cl_addr);

    int ret = ::recvfrom(_fd,
                         buffer,
                         size,
                         MSG_WAITALL,
                         (struct sockaddr *)&_cl_addr,
                         &cl_addr_size);

    if(ret == -1 &&
            (errno != EAGAIN || errno == EWOULDBLOCK))
    {
        perror("recvfrom failed");
    }

    return ret;
}

int UdpServer::reply(const uint8_t *buffer, size_t size)
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

std::string UdpServer::get_last_client_address() const
{
    std::string addr = inet_ntoa(_cl_addr.sin_addr);
    return addr + ":" + std::to_string(ntohs(_cl_addr.sin_port));
}

UdpClient::UdpClient()
{

}

bool UdpClient::init(std::string addr, int port)
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

int UdpClient::send(uint8_t *buffer, size_t size)
{
    socklen_t sv_addr_size = sizeof(_sv_addr);

    int ret = ::sendto(_fd,
                       buffer,
                       size,
                       MSG_CONFIRM,
                       (const struct sockaddr *)&_sv_addr,
                       sv_addr_size);

    if(ret == -1)
    {
        perror("sendto failed");
    }

    return ret;
}

int UdpClient::try_receive(uint8_t *buffer, size_t size)
{
    socklen_t sv_addr_size = sizeof(_sv_addr);

    int ret = ::recvfrom(_fd,
                         buffer,
                         size,
                         MSG_WAITALL,
                         (struct sockaddr *)&_sv_addr,
                         &sv_addr_size);

    if(ret == -1 &&
            (errno != EAGAIN || errno == EWOULDBLOCK))
    {
        perror("recvfrom failed");
    }

    return ret;
}
