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

static std::unique_ptr<com_util<fredo_msg>> feedback_subscriber, pulse_publisher;
volatile std::sig_atomic_t stop_flag;
static fredo_msg raw_from_down;
static std::mutex sub_mutex;

void func1()
{
    while (true)
    {
        // std::cout<<1<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 

        sub_mutex.lock();
        std::cout<<feedback_subscriber->data.pot_val_1<<std::endl;
        std::cout<<feedback_subscriber->data.pot_val_2<<std::endl;
        std::cout<<feedback_subscriber->data.pot_val_3<<std::endl<<std::endl;
        sub_mutex.unlock();

        
        // fredo_msg msg_lala;
        // msg_lala.time = 100600;
        // msg_lala.pot_val_1 = 100;
        // msg_lala.pot_val_2 = 200;
        // msg_lala.pot_val_3 = 300;
        
        // pulse_publisher->pub_msg(msg_lala);
    }    
}

int main() 
{
    pulse_publisher = std::make_unique<com_util<fredo_msg>>("192.168.1.255", PULSE_RAW_TOPIC, PUB);
    feedback_subscriber = std::make_unique<com_util<fredo_msg>>("192.168.1.255", POT_RAW_TOPIC, SUB);
    std::thread lala_thread(func1);
    lala_thread.join();    
    
    return 0;
}