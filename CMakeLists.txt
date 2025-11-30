cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

idf_build_set_property(MINIMAL_BUILD ON)

project(SubaruCarlinkAdapter
        VERSION 0.0.1
        DESCRIPTION "Subaru Carlink Adapter"
        HOMEPAGE_URL https://github.com/jadjer/subaru-carlink-adapter
        LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
