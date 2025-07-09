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
#include <vector>
#include <deque>

static std::unique_ptr<com_util<fredo_msg>> 
    feedback_subscriber, 
    pulse_publisher, 
    joint_cmd_subscriber,
    joint_deg_publisher;

volatile std::sig_atomic_t stop_flag;
static fredo_msg raw_from_down;
static std::mutex sub_mutex;
static fredo_msg q_state, q_cmd;
static std::deque<fredo_msg> q_state_buffer;

void mainloop();
void get_joint_deg();
void set_joint_deg();

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

void mainloop()
{
    while (true)
    {
        // std::cout<<1<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        get_joint_deg();
        
        
        // fredo_msg msg_lala;
        // msg_lala.time = 100600;
        // msg_lala.joint1 = 100;
        // msg_lala.joint2 = 200;
        // msg_lala.joint3 = 300;
        
        // pulse_publisher->pub_msg(msg_lala);
    }    
}

void get_joint_deg()
{
    sub_mutex.lock();

    q_state = feedback_subscriber->callback();

    // map q_cmd from pot_val to degree
    // here....
    // std::cout<<q_state.joint1<<std::endl;
    // std::cout<<q_state.joint1 - 64.0<<std::endl;
    // std::cout<<q_state.joint3<<std::endl<<std::endl;

    q_state.joint1 = (-1) * ((q_state.joint1 - 64) / (604 - 64) * 180.0 * (-1) + 180.0);
    q_state.joint2 = (q_state.joint2 - 86) / (614 - 86) * 180.0 * (-1) + 90.0;
    q_state.joint3 = (q_state.joint3 - 44) / (600 - 44) * 180.0 * (-1) + 90.0;

    // std::cout<<q_state.joint1<<std::endl;
    // std::cout<<q_state.joint2<<std::endl;
    // std::cout<<q_state.joint3<<std::endl<<std::endl;

    // low pass filter here
    q_state_buffer.emplace_back(q_state);
    
    sub_mutex.unlock();

    joint_deg_publisher->pub_msg(q_state);

}

void set_joint_deg()
{
    q_cmd = joint_cmd_subscriber->callback();
    // map q_cmd from deg to pulse
    // here....
    // q_cmd.joint1 = 
    // if (q_state_buffer.size() > 3)
    // {

    // }
    
    pulse_publisher->pub_msg(q_cmd);
}