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

auto Controller::readMessage() const -> std::expected<Message, MessageError> {
    Message message{};

    auto readField = [&](auto& field, std::size_t const bits) {
        auto const data = readData(bits);
        if (not data) {
            return false;
        }
        field = *data;
        return true;
    };

    auto handleAck = [&] { return not message.isBroadcast and message.slave == m_address ? writeAck() : skipAck(); };

    m_driver.waitStartBit();

    if (not readField(message.isBroadcast, 1)) {
        return std::unexpected(MessageError::BROADCAST_BIT_READ_ERROR);
    }

    if (not readField(message.master, 12)) {
        return std::unexpected(MessageError::MASTER_ADDRESS_READ_ERROR);
    }

    if (not readField(message.slave, 12)) {
        return std::unexpected(MessageError::SLAVE_ADDRESS_READ_ERROR);
    }
    if (not handleAck()) {
        return std::unexpected(MessageError::ACKNOWLEDGE_ERROR);
    }

    if (not readField(message.control, 4)) {
        return std::unexpected(MessageError::CONTROL_DATA_READ_ERROR);
    }
    if (not handleAck()) {
        return std::unexpected(MessageError::ACKNOWLEDGE_ERROR);
    }

    if (not readField(message.dataLength, 8)) {
        return std::unexpected(MessageError::DATA_LENGTH_READ_ERROR);
    }
    if (not handleAck()) {
        return std::unexpected(MessageError::ACKNOWLEDGE_ERROR);
    }

    for (std::size_t i = 0; i < message.dataLength; ++i) {
        if (not readField(message.data[i], 8)) {
            return std::unexpected(MessageError::DATA_READ_ERROR);
        }
        if (not handleAck()) {
            return std::unexpected(MessageError::ACKNOWLEDGE_ERROR);
        }
    }

    return message;
}

auto Controller::readBroadcastBit() const -> std::expected<bool, MessageError> {
    auto const optionalBit = m_driver.readBit();
    if (not optionalBit) {
        return false;
    }

    auto const bit = optionalBit.value();

    return bit;
}

auto Controller::readData(std::size_t const bitsCount) const -> std::expected<std::uint32_t, MessageError> {
    if (bitsCount > 32) {
        return std::unexpected(MessageError::BITS_COUNT_ERROR);
    }

    std::uint32_t data  = 0;
    std::uint8_t parity = 0;

    for (std::size_t i = 0; i < bitsCount; ++i) {
        auto const optionalBit = m_driver.readBit();
        if (not optionalBit) {
            return std::unexpected(MessageError::BUS_READ_ERROR);
        }

        auto const bit = optionalBit.value();

        data = data << 1 | bit;
        parity += bit;
    }

    if (not checkParity(parity)) {
        return std::unexpected(MessageError::BIT_PARITY_ERROR);
    }

    return data;
}

auto Controller::writeAck() const -> bool {
    auto const optionalBit = m_driver.readBit();
    if (not optionalBit) {
        return false;
    }

    auto const bit = optionalBit.value();

    if (bit == 1) {
        m_driver.writeBit(0);
    }

    return true;
}

auto Controller::skipAck() const -> bool {
    auto const optionalBit = m_driver.readBit();
    if (not optionalBit) {
        return false;
    }

    return true;
}

auto Controller::checkParity(std::uint8_t const parity) const -> bool {
    auto const optionalBit = m_driver.readBit();
    if (not optionalBit) {
        return false;
    }

    auto const bit = optionalBit.value();

    return parity % 2 == bit;
}
