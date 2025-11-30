//
// Created by jadjer on 29.11.2025.
//

#pragma once

#include <cstdint>
#include <optional>


/**
 * @class Driver
 * IEBus Driver
 */
class Driver {
public:
    using Bit = bool;
    using Pin = std::uint8_t;

public:
    Driver(Pin rx, Pin tx, Pin enable) noexcept;

public:
    /**
     * Enable IEBus transmitter
     */
    auto enable() const -> void;
    /**
     * Disable IEBus transmitter
     */
    auto disable() const -> void;

public:
    /**
     * IEBus transmitter enabled
     * @return bool
     */
    [[nodiscard]] auto isEnabled() const -> bool;
    /**
     * IEBus is low
     * @return bool
     */
    [[nodiscard]] auto isBusLow() const -> bool;
    /**
     * IEBus is high
     * @return bool
     */
    [[nodiscard]] auto isBusHigh() const -> bool;

public:
    /**
     * Wait until start bit
     */
    auto waitStartBit() const -> void;

public:
    /**
     * Read single bit from the IEBus
     * Variants: Start Bit, Bit 0, Bit 1
     * If the pulse duration does not match the specified ones, std::nullopt is returned.
     * @return Optional Bit Type
     */
    [[nodiscard]] auto readBit() const -> std::optional<Bit>;

public:
    /**
     * Write single bit to the IEBus
     * @param value Type of bit
     */
    auto writeBit(Bit value) const -> void;

private:
    /**
     * Wait until the IEBus logic level is low
     */
    auto waitBusLow() const -> void;
    /**
     * Wait until the IEBus logic level is high
     */
    auto waitBusHigh() const -> void;

private:
    Pin const m_rxPin;
    Pin const m_txPin;
    Pin const m_enablePin;
};
