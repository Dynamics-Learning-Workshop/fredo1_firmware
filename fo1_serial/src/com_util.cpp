#include "../include/com_util.h"

template<typename T>
com_util<T>::com_util(
    std::string ip,
    const int port,
    int type_of_com
) :
broadcast_ip_(ip), 
broadcast_port_(port), 
listen_port_(port)
{
    using namespace std;

    // auto mainloop = [this]() mutable
    // {
    //     while (true)
    //     {
    //         buffer_mutex.lock();
    //         data = buffer;
    //         // std::cout<<"hi"<<std::endl;
    //         buffer_mutex.unlock();
    //     }
    // };

    if (type_of_com == PUB)
        set_advertiser();
    else if (type_of_com == SUB)
    {
        std::thread sub_thread(&com_util::set_subscriber, this);
        // std::thread main_thread(mainloop);
        sub_thread.detach();
        // main_thread.detach();
    }
        
}

template<typename T>
void com_util<T>::pub_msg(const T& message)
{
    ssize_t sent = sendto(
        sock,
        static_cast<const void*>(&message),
        sizeof(T),
        0,
        (sockaddr*)&broadcastAddr,
        sizeof(broadcastAddr)
    );

    if (sent < 0)
        throw std::runtime_error("MSG SENT PUB...EXITING");
}

template<typename T>
void com_util<T>::set_advertiser()
{
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) 
        std::cout << "SOCKET-OPEN FAILED" << std::endl;

    int broadcastEnable = 1;
    if (
        setsockopt(
            sock, 
            SOL_SOCKET, 
            SO_BROADCAST, 
            &broadcastEnable, 
            sizeof(broadcastEnable)
        ) < 0
    ) 
    {
        std::cerr << "SOCKET-OPTION FAILED" << std::endl;
        close(sock);
    }

    memset(&broadcastAddr, 0, sizeof(broadcastAddr)); // memory set
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_port = htons(broadcast_port_);
    if (
        inet_aton(broadcast_ip_.c_str(), &broadcastAddr.sin_addr) == 0
    ) 
    {
        std::cerr << "BROADCAST IP INVALID" << std::endl;
        close(sock);
    }
}

template<typename T>
void com_util<T>::set_subscriber()
{
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "CREATING SOCKET FAILED" << std::endl;
        return;
    }

    serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(listen_port_);

    int reuse = 1;
    if (
        setsockopt(
            sock, 
            SOL_SOCKET, 
            SO_REUSEADDR, 
            &reuse, 
            sizeof(reuse)
        ) < 0
    ) 
    {
        std::cerr << "SETSOCKOPT FAILED" << std::endl;
        close(sock);
        return;
    }

    if (
        bind(
            sock, 
            (sockaddr*)&serverAddr, 
            sizeof(serverAddr)
        ) < 0
    ) 
    {
        std::cerr << "BINDING FAILED";
        close(sock);
        return;
    }
    
    clientAddrLen = sizeof(clientAddr);

    while (true) 
    {
        ssize_t len = recvfrom(
            sock, 
            &buffer, 
            1024 - 1, 
            0,
            (sockaddr*)&clientAddr, &clientAddrLen
        );
        
        if (len == sizeof(T)) 
        {
            buffer_mutex.lock();
            data = buffer;
            buffer_mutex.unlock();
            sub_init = true;
        }
    }
}

template<typename T>
T com_util<T>::callback()
{

    buffer_mutex.lock();
    T temp = buffer;
    buffer_mutex.unlock();
    return temp;
}
template class com_util<fredo_msg>;
