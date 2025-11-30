cmake_minimum_required(VERSION 3.22.1)

set(SOURCES
        main.cpp
        ie_bus/Driver.cpp
        ie_bus/Controller.cpp
)

idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS ".")