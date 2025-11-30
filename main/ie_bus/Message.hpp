//
// Created by jadjer on 29.11.2025.
//

#pragma once

#include <cstdint>

using Byte    = std::uint8_t;
using Address = std::uint16_t;

struct Message {
    bool isBroadcast;
    Address master;
    Address slave;
    Byte control;
    Byte dataLength;
    Byte data[32];
};
