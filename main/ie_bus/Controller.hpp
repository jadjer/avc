//
// Created by jadjer on 29.11.2025.
//

#pragma once

#include <expected>

#include "Driver.hpp"
#include "Message.hpp"


enum class MessageError {
    BUS_READ_ERROR,
    BIT_SEQUENCE_ERROR,
    BIT_PARITY_ERROR,
    BITS_COUNT_ERROR,
    START_BIT_READ_ERROR,
    BROADCAST_BIT_READ_ERROR,
    MASTER_ADDRESS_READ_ERROR,
    SLAVE_ADDRESS_READ_ERROR,
    CONTROL_DATA_READ_ERROR,
    DATA_LENGTH_READ_ERROR,
    DATA_READ_ERROR,
    ACKNOWLEDGE_ERROR,
};

/**
 * @class Controller
 * IEBus Controller
 */
class Controller {
public:
    Controller(Driver::Pin rx, Driver::Pin tx, Driver::Pin enable, Address address);

public:
    /**
     *
     */
    auto enable() const -> void;
    /**
     *
     */
    auto disable() const -> void;

public:
    /**
     *
     * @return
     */
    [[nodiscard]] auto isEnabled() const -> bool;

public:
    /**
     *
     * @return
     */
    [[nodiscard]] auto readMessage() const -> std::expected<Message, MessageError>;

private:
    /**
     *
     * @return
     */
    [[nodiscard]] auto readBroadcastBit() const -> std::expected<bool, MessageError>;
    /**
     *
     * @param bitsCount
     * @return
     */
    [[nodiscard]] auto readData(std::size_t bitsCount) const -> std::expected<std::uint32_t, MessageError>;

private:
    /**
     *
     * @return
     */
    [[nodiscard]] auto writeAck() const -> bool;
    /**
     *
     * @return
     */
    [[nodiscard]] auto skipAck() const -> bool;
    /**
     *
     * @param parity
     * @return
     */
    [[nodiscard]] auto checkParity(std::uint8_t parity) const -> bool;

private:
    Address const m_address;

private:
    Driver m_driver;
};
