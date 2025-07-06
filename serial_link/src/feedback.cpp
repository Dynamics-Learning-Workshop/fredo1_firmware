#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>

#include <chrono>
#include <thread> 

int main() {
    // Open for read and write
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
        std::cerr << "Failed to set terminal attributes\n";
        close(fd);
        return 1;
    }

    // Main loop
    char buf[100];
    std::string input;

    auto ctrl_lastrequest = std::chrono::high_resolution_clock::now();


    while (true) {
        // Read from Arduino
        int n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = 0;
            std::cout << "Received: " << buf << std::endl;
        } else if (n == 0 || (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // No data available, continue
        } else {
            perror("read error");
            break;
        }

        auto dt_raw = std::chrono::high_resolution_clock::now() - ctrl_lastrequest;
        auto dt = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(dt_raw);
        double elapsed_ms = dt.count();


        if (elapsed_ms > 20)
        {
            std::cout<<elapsed_ms<<std::endl;
            input = "1006\n";
            write(fd, input.c_str(), input.size());
            std::cout << "Sent: " << input << std::endl;   
            

            ctrl_lastrequest = std::chrono::high_resolution_clock::now();
        }             
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    close(fd);
    return 0;
}
