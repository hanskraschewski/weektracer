cmake_minimum_required(VERSION 3.10)

project(weektracer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set( SOURCES
    src/main.cpp
    src/color.h
    src/vec3.h
)

add_executable(weektracer src/main.cpp)