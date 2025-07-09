/*
    1. listen pot_val, talk joint_angle
    2. listen joint_cmd_angle, talk pulse
*/

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
static fredo_msg q_state, q_state_prev, q_cmd;
static double alpha = 0.1;
static std::deque<fredo_msg> q_state_buffer;
static std::pair<bool, fredo_msg> q_state_callback_obj, q_cmd_callback_obj;

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
    while (true) // running at 100 Hz
    {
        get_joint_deg();
        set_joint_deg();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
    }    
}

void get_joint_deg()
{
    if (!feedback_subscriber->sub_init)
        return;
    
    q_state_callback_obj = feedback_subscriber->callback();
    
    if (!q_state_callback_obj.first)
        return;

    q_state = q_state_callback_obj.second;
    
    q_state.joint1 = (-1) * ((q_state.joint1 - 64) / (604 - 64) * 180.0 * (-1) + 180.0);
    q_state.joint2 = (q_state.joint2 - 86) / (614 - 86) * 180.0 * (-1) + 90.0;
    q_state.joint3 = (q_state.joint3 - 44) / (600 - 44) * 180.0 * (-1) + 90.0;

    
    // low pass filter here
    q_state_buffer.emplace_back(q_state);

    if (q_state_buffer.size() < 5)
    {
        q_state_prev = q_state;
        return;
    }
            
    if (q_state_buffer.size() > 5)
        q_state_buffer.pop_front();

    fredo_msg q_temp;
    for (auto what : q_state_buffer)
    {
        q_temp.joint1 += what.joint1;
        q_temp.joint2 += what.joint2;
        q_temp.joint3 += what.joint3;
    }

    size_t buf_size = q_state_buffer.size();

    q_state.joint1 = alpha * q_temp.joint1 / buf_size + (1 - alpha) * q_state_prev.joint1;
    q_state.joint2 = alpha * q_temp.joint2 / buf_size + (1 - alpha) * q_state_prev.joint2;
    q_state.joint3 = alpha * q_temp.joint3 / buf_size + (1 - alpha) * q_state_prev.joint3;

    q_state_prev = q_state;

    joint_deg_publisher->pub_msg(q_state);
}

void set_joint_deg()
{
    if (!joint_cmd_subscriber->sub_init)
        return;
    q_cmd_callback_obj = joint_cmd_subscriber->callback();
    if (!q_cmd_callback_obj.first)
        return;

    auto now = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(now.time_since_epoch()).count();
    
    q_cmd.time = ms;
    q_cmd.joint1 = 2500 + q_cmd_callback_obj.second.joint1 / 180 * 2000;
    q_cmd.joint2 = 2500 - (q_cmd_callback_obj.second.joint2 + 90) / 180 * 2000;
    q_cmd.joint3 = 2500 - (q_cmd_callback_obj.second.joint3 + 90) / 180 * 2000;

    pulse_publisher->pub_msg(q_cmd);
}