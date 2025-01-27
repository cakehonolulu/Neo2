#include "neo2.hh"
#include <ee/dmac/dmac.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

EE_DMAC::EE_DMAC(Bus& bus) : bus(bus) {
    Logger::set_subsystem("EE_DMAC");
    // Initialize DMAC registers and channels
    D_CTRL = 0;
    D_STAT = 0;
    D_PCR = 0;
    D_SQWC = 0;
    D_RBSR = 0;
    D_RBOR = 0;
    D_ENABLER = 0;
    D_ENABLEW = 0;

    // Initialize channels with names
    channels[0x10008000] = { "VIF0", DMAC_Channel() };
    channels[0x10009000] = { "VIF1", DMAC_Channel() };
    channels[0x1000A000] = { "GIF", DMAC_Channel() };
    channels[0x1000B000] = { "IPU_FROM", DMAC_Channel() };
    channels[0x1000B400] = { "IPU_TO", DMAC_Channel() };
    channels[0x1000C000] = { "SIF0", DMAC_Channel() };
    channels[0x1000C400] = { "SIF1", DMAC_Channel() };
    channels[0x1000C800] = { "SIF2", DMAC_Channel() };
    channels[0x1000D000] = { "SPR_FROM", DMAC_Channel() };
    channels[0x1000D400] = { "SPR_TO", DMAC_Channel() };
}

void EE_DMAC::write_register(uint32_t address, uint32_t value) {
    uint32_t base_address = address & 0xFFFFF000;
    auto it = channels.find(base_address);

    if (it != channels.end()) {
        auto& [channel_name, channel] = it->second;
        const char* register_name = nullptr;

        switch (address & 0xFF) {
            case 0x00: register_name = "CHCR";
            {
                channel.CHCR = value;
                dma_step();
            }
            break;
            case 0x10: register_name = "MADR"; channel.MADR = value; break;
            case 0x20: register_name = "QWC"; channel.QWC = value; break;
            case 0x30: register_name = "TADR"; channel.TADR = value; break;
            case 0x40: register_name = "ASR0"; channel.ASR0 = value; break;
            case 0x50: register_name = "ASR1"; channel.ASR1 = value; break;
            case 0x80: register_name = "SADR"; channel.SADR = value; break;
            default:
                Logger::error("Invalid channel register address: 0x" + format("{:08X}", address));
                return;
        }

        Logger::info("DMAC write to channel " + channel_name + " register " + register_name + " at address 0x" + format("{:08X}", address) + " with value 0x" + format("{:08X}", value));
    } else {
        const char* global_register_name = nullptr;

        switch (address) {
            case 0x1000E000: global_register_name = "D_CTRL"; D_CTRL = value; break;
            case 0x1000E010: global_register_name = "D_STAT"; D_STAT = value; break;
            case 0x1000E020: global_register_name = "D_PCR"; D_PCR = value; break;
            case 0x1000E030: global_register_name = "D_SQWC"; D_SQWC = value; break;
            case 0x1000E040: global_register_name = "D_RBSR"; D_RBSR = value; break;
            case 0x1000E050: global_register_name = "D_RBOR"; D_RBOR = value; break;
            case 0x1000F590: global_register_name = "D_ENABLEW"; D_ENABLEW = value; break;
            default:
                Logger::error("Invalid DMAC address: 0x" + format("{:08X}", address));
                return;
        }

        Logger::info("DMAC write to global register " + std::string(global_register_name) + " at address 0x" + format("{:08X}", address) + " with value 0x" + format("{:08X}", value));
    }
}

uint32_t EE_DMAC::read_register(uint32_t address) {
    uint32_t base_address = address & 0xFFFFF000;
    auto it = channels.find(base_address);

    if (it != channels.end()) {
        const auto& [channel_name, channel] = it->second;
        const char* register_name = nullptr;
        uint32_t value = 0;

        switch (address & 0xFF) {
            case 0x00: register_name = "CHCR";
                {
                    value = channel.CHCR;
                    dma_step();
                }
                break;
            case 0x10: register_name = "MADR"; value = channel.MADR; break;
            case 0x20: register_name = "QWC"; value = channel.QWC; break;
            case 0x30: register_name = "TADR"; value = channel.TADR; break;
            case 0x40: register_name = "ASR0"; value = channel.ASR0; break;
            case 0x50: register_name = "ASR1"; value = channel.ASR1; break;
            case 0x80: register_name = "SADR"; value = channel.SADR; break;
            default:
                Logger::error("Invalid channel register address: 0x" + format("{:08X}", address));
                return 0;
        }

        Logger::info("DMAC read from channel " + channel_name + " register " + register_name + " at address 0x" + format("{:08X}", address) + " with value 0x" + format("{:08X}", value));
        return value;
    } else {
        const char* global_register_name = nullptr;
        uint32_t value = 0;

        switch (address) {
            case 0x1000E000: global_register_name = "D_CTRL"; value = D_CTRL; break;
            case 0x1000E010: global_register_name = "D_STAT"; value = D_STAT; break;
            case 0x1000E020: global_register_name = "D_PCR"; value = D_PCR; break;
            case 0x1000E030: global_register_name = "D_SQWC"; value = D_SQWC; break;
            case 0x1000E040: global_register_name = "D_RBSR"; value = D_RBSR; break;
            case 0x1000E050: global_register_name = "D_RBOR"; value = D_RBOR; break;
            case 0x1000F590: global_register_name = "D_ENABLEW"; value = D_ENABLEW; break;
            default:
                Logger::error("Invalid DMAC address: 0x" + format("{:08X}", address));
                return 0;
        }

        Logger::info("DMAC read from global register " + std::string(global_register_name) + " at address 0x" + format("{:08X}", address) + " with value 0x" + format("{:08X}", value));
        return value;
    }
}

DMAC_Channel* EE_DMAC::get_channel(uint32_t address) {
    uint32_t base_address = address & 0xFFFFF000;
    auto it = channels.find(base_address);
    if (it != channels.end()) {
        return &(it->second.second);
    }
    return nullptr;
}

void EE_DMAC::dma_step() {
    for (auto& [address, channel_data] : channels) {
        auto& [channel_name, channel] = channel_data;

        if (channel_name == "GIF") {
            // Check if the channel is active (CHCR.STR bit is set)
            if (channel.CHCR & 0x100) {
                process_gif_dma(channel);
            }
        }
    }
}

void EE_DMAC::process_gif_dma(DMAC_Channel& channel) {
    Logger::info("Starting GIF DMA transfer");
    uint32_t transfer_mode = (channel.CHCR >> 2) & 0x3;

    switch (transfer_mode) {
        case 0: // Burst mode
            Logger::info("Burst mode transfer for GIF DMA");
            process_burst_mode(channel);
            break;

        case 1: // Chain mode
            Logger::info("Unimplemented chain mode transfer for GIF DMA");
            Neo2::exit(1, Neo2::Subsystem::EE_DMAC);
            break;

        case 2: // Interleave mode (unimplemented)
            Logger::error("Unimplemented Interleave mode transfer for GIF DMA");
            Neo2::exit(1, Neo2::Subsystem::EE_DMAC);
            break;

        default:
            Logger::error("Unknown GIF DMA transfer mode: " + std::to_string(transfer_mode));
            Neo2::exit(1, Neo2::Subsystem::EE_DMAC);
            break;
    }

    // Clear the STR bit in CHCR to mark the channel as inactive
    channel.CHCR &= ~CHCR_STR_BIT;
}

void EE_DMAC::process_burst_mode(DMAC_Channel& channel) {
    Logger::info("Starting burst mode transfer, MADR: 0x" + format("{:08X}", channel.MADR) + ", QWC: " + std::to_string(channel.QWC));

    while (channel.QWC > 0) {
        uint128_t data = bus.read128(channel.MADR); // Read data from memory
        if (!bus.gif.is_path3_masked()) {
            bus.gif.write_fifo(data, channel.MADR, channel.CHCR); // Write data to GIF FIFO
        }
        else
        {
            Logger::warn("Burst mode PATH3 masked; ignoring GIF FIFO write");
        }
        channel.MADR += 16; // Update the memory address
        channel.QWC -= 1; // Decrease the transfer count
    }
}
