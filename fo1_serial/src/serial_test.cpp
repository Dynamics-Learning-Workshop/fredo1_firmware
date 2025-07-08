#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <memory>
#include "../include/com_util.h"

static std::unique_ptr<com_util<fredo_msg>> feedback_subscriber;

int main() 
{
    feedback_subscriber = std::make_unique<com_util<fredo_msg>>("192.168.1.255", 60000, SUB);
    
    return 0;
}