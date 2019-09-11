cmake_minimum_required(VERSION 3.0)

project(sbus)

set(SRCS
    src/SBUS.cpp
    src/main.cpp
    )

set(HEADERS
    src/SBUS.h
    )

add_library(sbus STATIC ${SRCS} ${HEADERS})
add_definitions(-D__RASPBERRYPI_ZERO_W__)
add_executable(main ${SRCS})
