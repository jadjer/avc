//
// Created by jadjer on 29.11.2025.
//

#include "Driver.hpp"

#include <chrono>
#include <driver/gpio.h>
#include <thread>

namespace {
    auto constexpr START_BIT_PULSE_WIDTH     = 170;
    auto constexpr BIT_0_PULSE_WIDTH         = 33;
    auto constexpr BIT_1_PULSE_WIDTH         = 20;
    auto constexpr BIT_PULSE_WIDTH_THRESHOLD = 5;
    auto constexpr START_BIT_MIN_PULSE_WIDTH = START_BIT_PULSE_WIDTH - BIT_PULSE_WIDTH_THRESHOLD;
    auto constexpr START_BIT_MAX_PULSE_WIDTH = START_BIT_PULSE_WIDTH + BIT_PULSE_WIDTH_THRESHOLD;
    auto constexpr BIT_0_MIN_PULSE_WIDTH     = BIT_0_PULSE_WIDTH - BIT_PULSE_WIDTH_THRESHOLD;
    auto constexpr BIT_0_MAX_PULSE_WIDTH     = BIT_0_PULSE_WIDTH + BIT_PULSE_WIDTH_THRESHOLD;
    auto constexpr BIT_1_MIN_PULSE_WIDTH     = BIT_1_PULSE_WIDTH - BIT_PULSE_WIDTH_THRESHOLD;
    auto constexpr BIT_1_MAX_PULSE_WIDTH     = BIT_1_PULSE_WIDTH + BIT_PULSE_WIDTH_THRESHOLD;
} // namespace

Driver::Driver(Pin const rx, Pin const tx, Pin const enable) noexcept : m_rxPin(rx), m_txPin(tx), m_enablePin(enable) {
    gpio_config_t const receiverConfiguration = {
        .pin_bit_mask = (1ULL << m_rxPin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
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

auto Driver::enable() const -> void {
    gpio_set_level(static_cast<gpio_num_t>(m_enablePin), 1);
}

auto Driver::disable() const -> void {
    gpio_set_level(static_cast<gpio_num_t>(m_enablePin), 0);
}

auto Driver::isEnabled() const -> bool {
    return gpio_get_level(static_cast<gpio_num_t>(m_enablePin));
}

auto Driver::isBusLow() const -> bool {
    return gpio_get_level(static_cast<gpio_num_t>(m_rxPin)) == 0;
}

auto Driver::isBusHigh() const -> bool {
    return gpio_get_level(static_cast<gpio_num_t>(m_rxPin)) == 1;
}

auto Driver::waitStartBit() const -> void {
    bool isStarted = false;

    while (not isStarted) {
        waitBusHigh();
        auto const pulseStartTime = std::chrono::high_resolution_clock::now();

        waitBusLow();
        auto const pulseStopTime = std::chrono::high_resolution_clock::now();

        auto const pulseTimeDelta = pulseStopTime - pulseStartTime;
        auto const pulseWidth     = std::chrono::duration_cast<std::chrono::microseconds>(pulseTimeDelta).count();

        if (pulseWidth >= START_BIT_MIN_PULSE_WIDTH and pulseWidth <= START_BIT_MAX_PULSE_WIDTH) {
            isStarted = true;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
}

auto Driver::readBit() const -> std::optional<Bit> {
    waitBusHigh();
    auto const pulseStartTime = std::chrono::high_resolution_clock::now();

    waitBusLow();
    auto const pulseStopTime = std::chrono::high_resolution_clock::now();

    auto const pulseTimeDelta = pulseStopTime - pulseStartTime;
    auto const pulseWidth     = std::chrono::duration_cast<std::chrono::microseconds>(pulseTimeDelta).count();

    if (pulseWidth >= BIT_0_MIN_PULSE_WIDTH and pulseWidth <= BIT_0_MAX_PULSE_WIDTH) {
        return 0;
    }
    if (pulseWidth >= BIT_1_MIN_PULSE_WIDTH and pulseWidth <= BIT_1_MAX_PULSE_WIDTH) {
        return 1;
    }

    return std::nullopt;
}

auto Driver::writeBit(Bit const value) const -> void {
    gpio_set_level(static_cast<gpio_num_t>(m_txPin), 1);

    bool isPulseCompleted = false;

    auto const pulseStartTime = std::chrono::high_resolution_clock::now();

    while (not isPulseCompleted) {
        std::this_thread::sleep_for(std::chrono::microseconds(1));

        auto const currentTime = std::chrono::high_resolution_clock::now();

        auto const pulseTimeDelta = currentTime - pulseStartTime;
        auto const pulseWidth     = std::chrono::duration_cast<std::chrono::microseconds>(pulseTimeDelta).count();

        if (value == 0 and pulseWidth >= BIT_0_PULSE_WIDTH) {
            isPulseCompleted = true;
        }
        if (value == 1 and pulseWidth >= BIT_1_PULSE_WIDTH) {
            isPulseCompleted = true;
        }
    }

    gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
}

auto Driver::waitBusLow() const -> void {
    while (isBusHigh()) {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
}

auto Driver::waitBusHigh() const -> void {
    while (isBusLow()) {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
}
