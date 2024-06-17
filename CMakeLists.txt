cmake_minimum_required(VERSION 3.10)

project(test_withoutros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(test_withoutros
    src/crc.cpp
    src/newDriver.cpp
    src/read.cpp
)