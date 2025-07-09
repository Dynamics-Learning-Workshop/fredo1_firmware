cmake_minimum_required(VERSION 3.10)
project(fo1_serial)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(serial_link 
    src/serial_link.cpp
    src/com_util.cpp
)

add_executable(serial_com 
    src/serial_com.cpp
    src/com_util.cpp
)

add_executable(ctrl_test 
    src/ctrl_test.cpp
    src/com_util.cpp
)
