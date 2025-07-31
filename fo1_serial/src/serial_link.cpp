#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>

#include <chrono>
#include <thread> 

#include <csignal>
#include <vector>
#include <sstream>
#include <algorithm>

#include <mutex>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring> 
#include <memory>

#include "../include/com_util.h"

// serial-related
volatile std::sig_atomic_t stop_flag;

void handle_sigint(int signum);
void joint_spinner();
void joints_callback(const char buf[]);
void process_pwm();
double chrono_to_double(const std::chrono::nanoseconds time_chrono);
bool is_digits(const std::string& s);

static std::vector<double> message_from_down;
static bool system_on = false, feedback_on = false;
static double dt_ms;
static int t_ard_ms;
static int fsm;
static int pots_raw[3];
static std::mutex pots_raw_mutex;
static std::mutex pulse_raw_mutex;
static std::mutex feedback_on_mutex;
static std::pair<bool, fredo_msg> pulse_callback_obj;

void serial_thread_func();

static std::unique_ptr<com_util<fredo_msg>> feedback_advertiser, pulse_subscriber;
void pub_thread_func();
static int mode;

int main(int argc, char* argv[])
{
    if (
        argv[1] == nullptr || argv[1][0] == '\0'
    ) 
    {
        std::cerr << "PLEASE ADVISE MODE!" << std::endl;
        return 0;
    }
    
    if (std::string(argv[1]) == "CTRL")
    {
        mode = CTRL_MODE;
        std::cout << "CTRL MODE" << std::endl;
    }
    else if (std::string(argv[1]) == "TWIN")
    {
        mode = TWIN_MODE;
        std::cout << "TWIN MODE" << std::endl;
    }
    else if (std::string(argv[1]) == "CALI")
    {
        mode = CALI_MODE;
        std::cout << "CALI MODE" << std::endl;
    }
    else
    {
        std::cerr << "MODE NODE SUPPORTED...END" << std::endl;
        return 0;
    }

    std::signal(SIGINT, handle_sigint);
    feedback_advertiser = std::make_unique<com_util<fredo_msg>>("192.168.1.255", POT_RAW_TOPIC, PUB);
    pulse_subscriber = std::make_unique<com_util<fredo_msg>>("192.168.1.255", PULSE_RAW_TOPIC, SUB);

    std::thread serial_thread(serial_thread_func);    
    std::thread pub_thread(pub_thread_func);
    
    serial_thread.join();
    pub_thread.join();

    
    return 0;
}


void handle_sigint(int signum)
{
    stop_flag = 1;
}

void joint_spinner()
{

}

void joints_callback(const char buf[])
{
    message_from_down.clear();
    std::string str(buf);         
    std::stringstream ss(str);           
    std::string token;

    feedback_on_mutex.lock();
    feedback_on = true;
    feedback_on_mutex.unlock();

    while (std::getline(ss, token, '-')) 
    {
        token.erase(std::remove_if(token.begin(), token.end(),
                           [](unsigned char c) { return c == '\n' || c == '\r'; }),
            token.end());

        if (
            is_digits(token)
            // true
        )
        {
            // std::cout<<token<< " ";
            message_from_down.emplace_back(std::stod(token));
        }
    }
        
            
    // std::cout <<std::endl << message_from_down.size()<<std::endl;

    t_ard_ms = message_from_down[0];
    fsm = message_from_down[1];

    pots_raw_mutex.lock();
    pots_raw[0] = message_from_down[2];
    pots_raw[1] = message_from_down[3];
    pots_raw[2] = message_from_down[4];
    pots_raw_mutex.unlock();

    return;
}

bool is_digits(const std::string& s) 
{
    try 
    {
        size_t idx;
        std::stod(s, &idx);  // try parsing to double
        return idx == s.size(); // ensure full string was parsed
    } 
    catch (...) 
    {
        return false;
    }
}


double chrono_to_double(const std::chrono::nanoseconds time_chrono)
{
    // here we return milli sec in double
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_chrono).count();
}

void serial_thread_func()
{
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK); 
    if (fd == -1) 
    {
        std::cerr << "SERIAL PORT OPENING FAILED" << std::endl;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) 
    {
        std::cerr << "TERMINAL ATTRIBUTES GET FAILED" << std::endl;
        close(fd);
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) 
    {
        std::cout << "FAILED TO OPEN /DEV/TTYACM0..." << std::endl;
        close(fd);
    }

    char buf[100];
    std::string input;
    auto ctrl_lastrequest = std::chrono::high_resolution_clock::now();

    fsm = 0;

    while (!stop_flag) 
    {
        // 1. first process buffer from serial
        int n = read(fd, buf, sizeof(buf) - 1);

        

        if (n > 0) 
        {
            buf[n] = 0;
            joints_callback(buf);
        } 
        else if (n == 0 || (errno == EAGAIN || errno == EWOULDBLOCK))         
            continue;
        else 
        {
            perror("read error");
            break;
        }
        
        // 2. infer which set are we at. we will only reach here if there is sth in buffer
    
        if (!system_on && fsm != 0)
        {
            std::cout<<"FREDO1 STARTED...\n\n";
            system_on = true;
        }
        
        switch (fsm)
        {
        case 0:
            // IDLE
            switch (mode)
            {
            case CTRL_MODE:
                input = "7777\n";
                break;

            case TWIN_MODE:
                input = "9999\n";
                break;

            case CALI_MODE:
                input = "1010\n";
                break;
            
            default:
                break;
            }
            write(fd, input.c_str(), input.size());
            break;

        case 1:
            // CTRL
            if (!pulse_subscriber->sub_init)
            {
                input = "1500-1500-1500\n";
                continue;
            }
            
            pulse_callback_obj = pulse_subscriber->callback();
            
            if (pulse_callback_obj.first)
                input = std::to_string(int(pulse_callback_obj.second.joint1)) + "-" +
                    std::to_string(int(pulse_callback_obj.second.joint2)) + "-" +
                    std::to_string(int(pulse_callback_obj.second.joint3)) + "\n";

            break;

        case 2:
            // twinning
            // input = "9999\n";
            // only twinning
            // write(fd, input.c_str(), input.size());
            break;
        
        default:
            break;
        }
        
        dt_ms = chrono_to_double(std::chrono::high_resolution_clock::now() - ctrl_lastrequest);
        if (dt_ms > 20 && (fsm == 1 || fsm == 3))
        {           
            write(fd, input.c_str(), input.size());
            ctrl_lastrequest = std::chrono::high_resolution_clock::now();
        }             
    }

    // exit loop
    while (system_on)
    {        
        input = "8888\n";
        write(fd, input.c_str(), input.size());

        int n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) 
        {
            buf[n] = 0;
            joints_callback(buf);
        } 
        else if (n == 0 || (errno == EAGAIN || errno == EWOULDBLOCK))         
            continue;
        else 
        {
            perror("read error");
            break;
        }

        if (fsm == 0)
            break;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));        
    }
    
    std::cout<<"FREDO1 SHUTDOWN...\n\n"<<std::endl;
    close(fd);
}

void pub_thread_func()
{
    while (!stop_flag)
    {
        pots_raw_mutex.lock();

        fredo_msg msg_lala;

        msg_lala.time = time_util::get_time();
        msg_lala.joint1 = pots_raw[0];
        msg_lala.joint2 = pots_raw[1];
        msg_lala.joint3 = pots_raw[2];
        pots_raw_mutex.unlock(); 
        
        feedback_on_mutex.lock();
        if (feedback_on)
            feedback_advertiser->pub_msg(msg_lala);
        feedback_on_mutex.unlock();
        
        // pub raw at 200 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  
    }        
}
