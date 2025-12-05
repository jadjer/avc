cmake_minimum_required(VERSION 3.22.1)

set(INCLUDES
        "."
)

set(SOURCES
        main.cpp

        Usb.cpp
)

set(PRIV_REQUIRES
        esp_iebus
        esp_tinyusb
)

idf_component_register(INCLUDE_DIRS ${INCLUDES} SRCS ${SOURCES}  PRIV_REQUIRES ${PRIV_REQUIRES})
