cmake_minimum_required(VERSION 3.10)
project(serial_link)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(serial_link src/serial_link.cpp)
