#pragma once

#include <memory>
#include <reg.hh>
#include <cstdint>
#include <array>

class VU {
public:
    VU(uint32_t instruction_memory_size, uint32_t data_memory_size);
    ~VU();

    void reset();
    void step();

    // Vector floating-point registers
    uint128_t vf[32];

    // Integer registers
    uint16_t vi[16];

    // Special registers
    uint128_t acc; // Accumulator
    uint32_t q;       // Q register
    uint32_t p;       // P register

    // MAC and Clip flags
    uint16_t mac_flags;
    uint32_t clip_flags;
    uint32_t status_flags;

private:
    std::unique_ptr<uint8_t[]> instruction_memory;
    std::unique_ptr<uint8_t[]> data_memory;

    uint32_t instruction_memory_size;
    uint32_t data_memory_size;
};

