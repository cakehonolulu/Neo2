#pragma once
#include <cstdint>
#include <array>

class IOP_Timer {
public:
    static constexpr uint32_t TIMER_COUNT_ADDR[6] = {0x1F801100, 0x1F801110, 0x1F801120, 0x1F801480, 0x1F801490, 0x1F8014A0};
    static constexpr uint32_t TIMER_MODE_ADDR[6] = {0x1F801104, 0x1F801114, 0x1F801124, 0x1F801484, 0x1F801494, 0x1F8014A4};
    static constexpr uint32_t TIMER_COMP_ADDR[6] = {0x1F801108, 0x1F801118, 0x1F801128, 0x1F801488, 0x1F801498, 0x1F8014A8};

    IOP_Timer();

    // Methods to handle register reads and writes
    uint32_t read(uint32_t address);
    void write(uint32_t address, uint32_t value);

private:
    struct TimerRegisters {
        uint32_t count;
        uint32_t mode;
        uint32_t target;
    };

    std::array<TimerRegisters, 6> timers; // Six hardware timers
};

