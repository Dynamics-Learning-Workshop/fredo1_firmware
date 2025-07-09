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

static std::unique_ptr<com_util<fredo_msg>> 
    feedback_subscriber, 
    pulse_publisher, 
    joint_cmd_subscriber,
    joint_deg_publisher;

volatile std::sig_atomic_t stop_flag;
static fredo_msg raw_from_down;
static std::mutex sub_mutex;

void mainloop()
{
    while (true)
    {
        // std::cout<<1<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 

        sub_mutex.lock();
        std::cout<<feedback_subscriber->callback().joint1<<std::endl;
        std::cout<<feedback_subscriber->callback().joint2<<std::endl;
        std::cout<<feedback_subscriber->callback().joint3<<std::endl<<std::endl;
        sub_mutex.unlock();
        
        // fredo_msg msg_lala;
        // msg_lala.time = 100600;
        // msg_lala.joint1 = 100;
        // msg_lala.joint2 = 200;
        // msg_lala.joint3 = 300;
        
        // pulse_publisher->pub_msg(msg_lala);
    }    
}

int main() 
{
    // ctrl related
    pulse_publisher = std::make_unique<com_util<fredo_msg>>("192.168.1.255", PULSE_RAW_TOPIC, PUB);
    joint_cmd_subscriber = std::make_unique<com_util<fredo_msg>>("192.168.1.255", JOINT_CMD_TOPIC, SUB);
    
    // feedback related
    feedback_subscriber = std::make_unique<com_util<fredo_msg>>("192.168.1.255", POT_RAW_TOPIC, SUB);
    joint_deg_publisher = std::make_unique<com_util<fredo_msg>>("192.168.1.255", JOINT_DEG_TOPIC, PUB);

    // threading
    std::thread lala_thread(mainloop);
    lala_thread.join();    
    
    return 0;
}