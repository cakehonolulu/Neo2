#pragma once
#include <cstdint>
#include <array>

class EE_Timer {
public:
    static constexpr uint32_t TIMER_COUNT_ADDR[4] = {0x10000000, 0x10000800, 0x10001000, 0x10001800};
    static constexpr uint32_t TIMER_MODE_ADDR[4] = {0x10000010, 0x10000810, 0x10001010, 0x10001810};
    static constexpr uint32_t TIMER_COMP_ADDR[4] = {0x10000020, 0x10000820, 0x10001020, 0x10001820};
    static constexpr uint32_t TIMER_HOLD_ADDR[2] = {0x10000030, 0x10000830}; // Only for T0 and T1

    EE_Timer();

    // Methods to handle register reads and writes
    uint32_t read(uint32_t address);
    void write(uint32_t address, uint32_t value);

private:
    struct TimerRegisters {
        uint16_t count;
        uint16_t comp;
        uint16_t hold;   // Only for T0 and T1
        uint32_t mode;
    };

    std::array<TimerRegisters, 4> timers; // Four hardware timers
};
