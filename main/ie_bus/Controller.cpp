//
// Created by jadjer on 29.11.2025.
//

#include "Controller.hpp"

Controller::Controller(Driver::Pin const rx, Driver::Pin const tx, Driver::Pin const enable, Address const address)
    : m_address(address), m_driver(rx, tx, enable) {}

auto Controller::enable() const -> void {
    m_driver.enable();
}

auto Controller::disable() const -> void {
    m_driver.disable();
}

auto Controller::isEnabled() const -> bool {
    return m_driver.isEnabled();
}

auto Controller::readMessage() const -> std::optional<Message> {
    if (auto const isStartBit = readStartBit(); not isStartBit) {
        return std::nullopt;
    }

    Message message{};

    auto readField = [this](auto& field, std::size_t const bits) {
        auto const data = readData(bits);
        if (not data) {
            return false;
        }
        field = *data;
        return true;
    };

    auto handleAck = [&] { return not message.isBroadcast and message.slave == m_address ? writeAck() : skipAck(); };

    if (not readField(message.isBroadcast, 1)) {
        return std::nullopt;
    }

    if (not readField(message.master, 12)) {
        return std::nullopt;
    }

    if (not readField(message.slave, 12)) {
        return std::nullopt;
    }
    if (not handleAck()) {
        return std::nullopt;
    }

    if (not readField(message.control, 4)) {
        return std::nullopt;
    }
    if (not handleAck()) {
        return std::nullopt;
    }

    if (not readField(message.dataLength, 8)) {
        return std::nullopt;
    }
    if (not handleAck()) {
        return std::nullopt;
    }

    for (std::size_t i = 0; i < message.dataLength; ++i) {
        if (not readField(message.data[i], 8)) {
            return std::nullopt;
        }
        if (not handleAck()) {
            return std::nullopt;
        }
    }

    return message;
}

auto Controller::readStartBit() const -> bool {
    auto const bit = m_driver.readBit();
    if (not bit) {
        return false;
    }
    if (bit != BitType::START_BIT) {
        return false;
    }
    return true;
}

auto Controller::readBroadcastBit() const -> std::expected<bool, MessageError> {
    auto const bit = m_driver.readBit();
    if (not bit) {
        return std::unexpected(MessageError::BUS_READ_ERROR);
    }
    if (bit == BitType::BIT_0) {
        return true;
    }
    if (bit == BitType::BIT_1) {
        return false;
    }
    return std::unexpected(MessageError::BIT_SEQUENCE_ERROR);
}

auto Controller::readData(std::size_t const bitsCount) const -> std::expected<std::uint32_t, MessageError> {
    if (bitsCount > 32) {
        return std::unexpected(MessageError::BITS_COUNT_ERROR);
    }

    std::uint32_t data  = 0;
    std::uint8_t parity = 0;

    for (std::size_t i = 0; i < bitsCount; ++i) {
        auto const bit = m_driver.readBit();
        if (not bit) {
            return std::unexpected(MessageError::BUS_READ_ERROR);
        }
        if (bit != BitType::BIT_0 and bit != BitType::BIT_1) {
            return std::unexpected(MessageError::BIT_SEQUENCE_ERROR);
        }
        if (bit == BitType::BIT_0) {
            data = data << 1 | 0;
        }
        if (bit == BitType::BIT_1) {
            data = data << 1 | 1;
            ++parity;
        }
    }

    if (not checkParity(parity)) {
        return std::unexpected(MessageError::BIT_PARITY_ERROR);
    }

    return data;
}

auto Controller::writeAck() const -> bool {
    auto const bit = m_driver.readBit();
    if (not bit) {
        return false;
    }
    if (bit == BitType::BIT_0) {
        return true;
    }
    if (bit == BitType::BIT_1) {
        m_driver.writeBit(BitType::BIT_0);
        return true;
    }
    return false;
}

auto Controller::skipAck() const -> bool {
    auto const bit = m_driver.readBit();
    if (not bit) {
        return false;
    }
    if (bit == BitType::BIT_0) {
        return true;
    }
    if (bit == BitType::BIT_1) {
        return true;
    }
    return false;
}

auto Controller::checkParity(std::uint8_t const parity) const -> bool {
    auto const bit = m_driver.readBit();
    if (not bit) {
        return false;
    }
    if (bit == BitType::BIT_0) {
        return parity % 2 != 0;
    }
    if (bit == BitType::BIT_1) {
        return parity % 2 != 1;
    }
    return false;
}
