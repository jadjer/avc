//
// Created by jadjer on 29.11.2025.
//

#pragma once

#include <optional>

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
     * Enable IEBus driver
     */
    auto enable() -> void;
    /**
     * Enable IEBus driver
     */
    auto disable() -> void;

public:
    /**
     *
     * @return
     */
    [[nodiscard]] auto isEnabled() const -> bool;

public:
    /**
     * Read message from IEBus
     * @return Optional message
     */
    [[nodiscard]] auto readMessage() const -> std::optional<Message>;
    /**
     * Write message to IEBus
     * @param message Message
     * @return bool
     */
    [[nodiscard]] auto writeMessage(Message const& message) const -> bool;

private:
    /**
     * Check parity for calculated parity
     * @param data Data for check
     * @param size Data size
     * @param parity Etalon parity
     * @return Comparison result
     */
    static auto checkParity(Data data, Size size, Bit parity) -> Bit;
    /**
     * Calculate parity for calculated parity
     * @param data Data for calculate
     * @param size Data size
     * @return Parity bit
     */
    static auto calculateParity(Data data, Size size) -> Bit;

private:
    Address const m_address;

private:
    Driver m_driver;
};
