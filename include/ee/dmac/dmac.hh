#pragma once

#include <cstdint>
#include <unordered_map>
#include <string>

// Channel struct to hold DMAC channel registers
struct DMAC_Channel {
    uint32_t CHCR; // Channel Control
    uint32_t MADR; // Channel Address
    uint32_t QWC;  // Quadword Count
    uint32_t TADR; // Tag Address
    uint32_t ASR0; // Saved Tag Address 0
    uint32_t ASR1; // Saved Tag Address 1
    uint32_t SADR; // Scratchpad Address
};

// DMAC class
class EE_DMAC {
public:
    EE_DMAC();

    void write_register(uint32_t address, uint32_t value);
    uint32_t read_register(uint32_t address);

private:
    std::unordered_map<uint32_t, DMAC_Channel> channels;
    uint32_t D_CTRL; // DMAC control
    uint32_t D_STAT; // DMAC interrupt status
    uint32_t D_PCR;  // DMAC priority control
    uint32_t D_SQWC; // DMAC skip quadword
    uint32_t D_RBSR; // DMAC ringbuffer size
    uint32_t D_RBOR; // DMAC ringbuffer offset
    uint32_t D_ENABLER; // DMAC disabled status
    uint32_t D_ENABLEW; // DMAC disable

    // Helper function to get channel by address
    DMAC_Channel *get_channel(uint32_t address);
};
