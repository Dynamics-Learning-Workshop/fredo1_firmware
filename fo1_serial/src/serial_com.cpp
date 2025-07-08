#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <memory>
#include "../include/com_util.h"
#include <csignal>
#include <chrono>
#include <thread>
#include <mutex>

static std::unique_ptr<com_util<fredo_msg>> feedback_subscriber;
volatile std::sig_atomic_t stop_flag;
static fredo_msg raw_from_down;
static std::mutex sub_mutex;

void func1()
{
    // sub_mutex.lock();
    // feedback_subscriber = std::make_unique<com_util<fredo_msg>>("192.168.1.255", 60000, SUB);
    while (true)
    {
        std::cout<<1<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  
        std::cout<<feedback_subscriber->data.pot_val_1<<std::endl;
    }    
}

int main() 
{
    feedback_subscriber = std::make_unique<com_util<fredo_msg>>("192.168.1.255", 60000, SUB);
    std::thread lala_thread(func1);
    lala_thread.join();    
    
    return 0;
}