#pragma once

#include <reg.hh>
#include <cstdint>
#include <unordered_map>
#include <string>
#include <utility>

class Bus; // Forward declaration of Bus class

// Channel struct to hold DMAC channel registers
struct DMAC_Channel {
    uint32_t CHCR = 0; // Channel Control
    uint32_t MADR = 0; // Channel Address
    uint32_t QWC = 0;  // Quadword Count
    uint32_t TADR = 0; // Tag Address
    uint32_t ASR0 = 0; // Saved Tag Address 0
    uint32_t ASR1 = 0; // Saved Tag Address 1
    uint32_t SADR = 0; // Scratchpad Address
};

// DMAC class
class EE_DMAC {
public:
    EE_DMAC(Bus& bus);

    void write_register(uint32_t address, uint32_t value);
    uint32_t read_register(uint32_t address);
    void dma_step();

    uint32_t D_CTRL = 0;    // DMAC control
    uint32_t D_STAT = 0;    // DMAC interrupt status
    uint32_t D_PCR = 0;     // DMAC priority control
    uint32_t D_SQWC = 0;    // DMAC skip quadword
    uint32_t D_RBSR = 0;    // DMAC ringbuffer size
    uint32_t D_RBOR = 0;    // DMAC ringbuffer offset
    uint32_t D_STADR = 0;   // DMAC stall address
    uint32_t D_ENABLER = 0; // DMAC disabled status
    uint32_t D_ENABLEW = 0; // DMAC disable

private:
    Bus& bus; // Reference to the Bus instance

    // Map of base addresses to channel name and DMAC_Channel
    std::unordered_map<uint32_t, std::pair<std::string, DMAC_Channel>> channels;

    static constexpr uint32_t CHCR_STR_BIT = 0x100; // Start bit for CHCR
    static constexpr uint32_t CHCR_ASR_BIT = 0x10;  // Address stack register select bit

    // Helper function to get channel by address
    DMAC_Channel* get_channel(uint32_t address);

    // Helper functions for GIF DMA
    void process_gif_dma(DMAC_Channel& channel);
    void process_burst_mode(DMAC_Channel& channel);
    void process_chain_mode(DMAC_Channel& channel);
    void handle_chain_tag(DMAC_Channel& channel, uint32_t tag_id, uint32_t addr, uint32_t qwc, bool& tag_end);
};
