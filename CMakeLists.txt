cmake_minimum_required(VERSION 3.22.1)

set(PICO_BOARD pico)
set(PICO_PLATFORM rp2040)
set(CMAKE_CXX_STANDARD 20)

include(cmake/pico_sdk_import.cmake)

project(avc)

pico_sdk_init()

add_subdirectory(src)
