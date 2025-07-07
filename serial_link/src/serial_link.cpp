#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>

#include <chrono>
#include <thread> 

#include <csignal>

volatile std::sig_atomic_t stop_flag;

void handle_sigint(int signum);
void joint_callback();
void joint_spinner();
void process_pot(const char buf[]);
void process_pwm();

static bool system_start = false;

int main() {

    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK); 
    if (fd == -1) {
        std::cerr << "Failed to open serial port\n";
        return 1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Failed to get terminal attributes\n";
        close(fd);
        return 1;
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

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cout << "FAILED TO OPEN /DEV/TTYACM0..." << std::endl;
        close(fd);
        return 1;
    }

    char buf[100];
    std::string input;
    auto ctrl_lastrequest = std::chrono::high_resolution_clock::now();

    std::signal(SIGINT, handle_sigint);

    while (!stop_flag) 
    {
        int n = read(fd, buf, sizeof(buf) - 1);

        if (n > 0) 
        {
            buf[n] = 0;
            process_pot(buf);
        } 
        else if (n == 0 || (errno == EAGAIN || errno == EWOULDBLOCK))         
            continue;
        else 
        {
            perror("read error");
            break;
        }
        
        auto dt_raw = std::chrono::high_resolution_clock::now() - ctrl_lastrequest;
        auto dt = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(dt_raw);
        double elapsed_ms = dt.count();

        if (elapsed_ms > 20 && system_start)
        {
            input = "1006\n";
            write(fd, input.c_str(), input.size());

            ctrl_lastrequest = std::chrono::high_resolution_clock::now();
        }             
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout<<"END!"<<std::endl;
    close(fd);

    
    return 0;
}


void handle_sigint(int signum)
{
    stop_flag = 1;
}

void joint_callback()
{

}

void joint_spinner()
{

}

void process_pot(const char buf[])
{
    std::cout << "Received here in small function: " << buf << std::endl;
}

