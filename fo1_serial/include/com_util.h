#include <mutex>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring> 
#include <iostream>
#include "fredo_msg.h"

#define SUB 0
#define PUB 1



template<typename T>
class com_util
{
private:
    int sock;
    std::string broadcast_ip_ = "192.168.1.255";
    int broadcast_port_ = 60000;
    int listen_port_ = 60000;
    sockaddr_in broadcastAddr;
    sockaddr_in serverAddr;
    sockaddr_in clientAddr;
    socklen_t clientAddrLen;

public:
    // objects

    // funcs
    com_util(
        std::string broadcast_ip,
        const int broadcast_port,
        int type_of_com
    );
    bool pub_msg(const T& message);
    bool sub_msg(const T& message);

    void set_advertiser();
    void set_subscriber();
    ~com_util()
    {
        close(sock);
    };
};

