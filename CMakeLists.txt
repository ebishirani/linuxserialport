cmake_minimum_required(VERSION 3.8.1)
project(linuxserialport)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++14")

include_directories(
    include
    )

add_library(${PROJECT_NAME} SHARED
        src/SerialPort.cpp)
