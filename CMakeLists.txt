cmake_minimum_required(VERSION 3.10)

project(weektracer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenMP REQUIRED)

set( SOURCES
    src/main.cpp
    src/color.h
    src/vec3.h
)

add_executable(weektracer src/main.cpp)

if(OpenMP_CXX_FOUND)
    target_link_libraries(weektracer PRIVATE OpenMP::OpenMP_CXX)
endif()