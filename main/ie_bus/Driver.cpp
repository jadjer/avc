//
// Created by jadjer on 29.11.2025.
//

#include "Driver.hpp"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>

namespace {
    auto constexpr TAG = "IEBusDriver";

    Driver::Time constexpr START_BIT_TOTAL_US     = 190;
    Driver::Time constexpr START_BIT_HIGH_US      = 171;
    Driver::Time constexpr START_BIT_LOW_US       = START_BIT_TOTAL_US - START_BIT_HIGH_US;
    Driver::Time constexpr START_BIT_THRESHOLD_US = 20;

    Driver::Time constexpr DATA_BIT_TOTAL_US  = 39;
    Driver::Time constexpr DATA_BIT_0_HIGH_US = 33;
    Driver::Time constexpr DATA_BIT_0_LOW_US  = DATA_BIT_TOTAL_US - DATA_BIT_0_HIGH_US;
    Driver::Time constexpr DATA_BIT_1_HIGH_US = 20;
    Driver::Time constexpr DATA_BIT_1_LOW_US  = DATA_BIT_TOTAL_US - DATA_BIT_1_HIGH_US;

    Driver::Time getTimeUs() {
        return esp_timer_get_time();
    }

    void delayUs(Driver::Time const delay) {
        auto const startTime = getTimeUs();

        bool enable = true;
        while (enable) {
            auto const currentTime    = getTimeUs();
            auto const differenceTime = currentTime - startTime;
            auto const isTimeOut      = differenceTime >= delay;

            if (isTimeOut) {
                enable = false;
            }
        }
    }

    Bit decodeBit(Driver::Time const pulseWidthUs) {
        Driver::Time const diff0 = std::abs(pulseWidthUs - DATA_BIT_0_HIGH_US);
        Driver::Time const diff1 = std::abs(pulseWidthUs - DATA_BIT_1_HIGH_US);

        if (diff1 < diff0) {
            return 1;
        }

        return 0;
    }
} // namespace

Driver::Driver(Pin const rx, Pin const tx, Pin const enable) noexcept : m_rxPin(rx), m_txPin(tx), m_enablePin(enable) {

    gpio_config_t const receiverConfiguration = {
        .pin_bit_mask = (1ULL << m_rxPin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&receiverConfiguration);

    gpio_config_t const transmitterConfiguration = {
        .pin_bit_mask = (1ULL << m_txPin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&transmitterConfiguration);

    gpio_config_t const enableConfiguration = {
        .pin_bit_mask = (1ULL << m_enablePin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&enableConfiguration);
}

auto Driver::isEnabled() const -> bool {
    return m_isEnabled;
}

auto Driver::isBusHigh() const -> bool {
    return gpio_get_level(static_cast<gpio_num_t>(m_rxPin));
}

auto Driver::isBusLow() const -> bool {
    return not isBusHigh();
}

auto Driver::isBusFree() const -> bool {
    if (isBusHigh()) {
        return false;
    }

    auto const startTime = getTimeUs();

    while (isBusLow()) {
        auto const currentTime    = getTimeUs();
        auto const timeDifference = currentTime - startTime;
        if (timeDifference >= DATA_BIT_TOTAL_US) {
            return true;
        }
    }

    return false;
}

auto Driver::enable() -> void {
    m_isEnabled = true;

    gpio_set_level(static_cast<gpio_num_t>(m_enablePin), m_isEnabled);
}

auto Driver::disable() -> void {
    m_isEnabled = false;

    gpio_set_level(static_cast<gpio_num_t>(m_enablePin), m_isEnabled);
}

auto Driver::receiveStartBit() const -> bool {
    auto constexpr startBitMinHighUs = START_BIT_HIGH_US - START_BIT_THRESHOLD_US;
    auto constexpr startBitMaxHighUs = START_BIT_HIGH_US + START_BIT_THRESHOLD_US;

    waitBusHigh();

    auto const startTime = getTimeUs();

    waitBusLow();

    auto const stopTime     = getTimeUs();
    auto const highDuration = stopTime - startTime;
    auto const isStartBit   = highDuration >= startBitMinHighUs and highDuration <= startBitMaxHighUs;

    return isStartBit;
}

auto Driver::receiveBit() const -> Bit {
    waitBusHigh();

    auto const startTime = getTimeUs();

    waitBusLow();

    auto const stopTime     = getTimeUs();
    auto const highDuration = stopTime - startTime;
    auto const bit          = decodeBit(highDuration);

    return bit;
}

auto Driver::receiveBits(Size const numBits) const -> Data {
    Data result = 0;

    for (Size i = 0; i < numBits; ++i) {
        auto const bit      = receiveBit();
        auto const bitValue = bit ? 1 : 0;
        auto const bitShift = numBits - 1 - i;

        result |= bitValue << bitShift;
    }

    return result;
}

auto Driver::receiveAckBit() const -> AcknowledgmentType {
    auto const ackBit = receiveBit();
    if (ackBit == 0) {
        return AcknowledgmentType::ACK;
    }

    return AcknowledgmentType::NAK;
}

auto Driver::transmitStartBit() const -> void {
    gpio_set_level(static_cast<gpio_num_t>(m_txPin), 1);
    delayUs(START_BIT_HIGH_US);

    gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
    delayUs(START_BIT_LOW_US);
}

auto Driver::transmitBit(Bit const bit) const -> void {
    Time const highDuration = bit ? DATA_BIT_1_HIGH_US : DATA_BIT_0_HIGH_US;
    Time const lowDuration  = bit ? DATA_BIT_1_LOW_US : DATA_BIT_0_LOW_US;

    gpio_set_level(static_cast<gpio_num_t>(m_txPin), 1);
    delayUs(highDuration);

    gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
    delayUs(lowDuration);
}

auto Driver::transmitBits(Data const data, Size const numBits) const -> void {
    for (Size i = 0; i < numBits; ++i) {
        Size const bitPosition = numBits - 1 - i;
        Bit const bit          = static_cast<Bit>(data >> bitPosition & 1);

        transmitBit(bit);
    }
}

auto Driver::sendAckBit(AcknowledgmentType const ack) const -> void {
    if (ack == AcknowledgmentType::ACK) {
        return transmitBit(0);
    }

    transmitBit(1);
}

auto Driver::waitBusLow() const -> void {
    while (isBusHigh()) {
    }
}

auto Driver::waitBusHigh() const -> void {
    while (isBusLow()) {
    }
}
