#include "neo2.hh"
#include <ee/dmac/dmac.hh>
#include <log/log.hh>
#include <fstream>

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

        //Logger::info("DMAC write to channel " + channel_name + " register " + register_name + " at address 0x" + format("{:08X}", address) + " with value 0x" + format("{:08X}", value));
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

        //Logger::info("DMAC write to global register " + std::string(global_register_name) + " at address 0x" + format("{:08X}", address) + " with value 0x" + format("{:08X}", value));
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

        //Logger::info("DMAC read from channel " + channel_name + " register " + register_name + " at address 0x" + format("{:08X}", address) + " with value 0x" + format("{:08X}", value));
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

        //Logger::info("DMAC read from global register " + std::string(global_register_name) + " at address 0x" + format("{:08X}", address) + " with value 0x" + format("{:08X}", value));
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
    //printf("Stepping DMA\n");
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
    //Logger::info("Starting GIF DMA transfer");
    uint32_t transfer_mode = (channel.CHCR >> 2) & 0x3;

    switch (transfer_mode) {
        case 0: // Burst mode
            //Logger::info("Burst mode transfer for GIF DMA");
            process_burst_mode(channel);
            break;

        case 1: // Chain mode
            process_chain_mode(channel);
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
    //Logger::info("Starting burst mode transfer, MADR: 0x" + format("{:08X}", channel.MADR) + ", QWC: " + std::to_string(channel.QWC));

    while (channel.QWC > 0) {
        uint128_t data = bus.read128(channel.MADR); // Read data from memory
        if (!bus.gif.is_path3_masked()) {
            //Logger::debug("Writing to GIF, current QWC: 0x" + format("{:d}", channel.QWC));
            bus.gif.write_fifo(data, channel.MADR, channel.QWC, false); // Write data to GIF FIFO
        }
        else
        {
            Logger::warn("Burst mode PATH3 masked; ignoring GIF FIFO write");
        }
        channel.MADR += 16; // Update the memory address
        channel.QWC -= 1; // Decrease the transfer count
    }
}

// Add a new helper to parse a DMAtag from a 128-bit value.
struct DMAtag {
    uint16_t qwc;    // bits  0-15: number of quadwords to transfer
    uint8_t tag_id;  // bits 28-30: tag ID
    bool irq;        // bit     31: IRQ flag
    uint32_t addr;   // bits 32-62: transfer address (lower 4 bits must be zero)
    // (Bits 63 and bits 64-127, used for memory select and optional tag data, are ignored here.)
};

static DMAtag parse_dmatag(uint128_t tag)
{
    // Assume that tag.u64[0] holds the lower 64 bits and tag.u64[1] the upper 64 bits.
    uint64_t low = tag.u64[0];
    DMAtag dt;
    dt.qwc    = low & 0xFFFF;            // bits  0-15
    dt.tag_id = (low >> 28) & 0x7;         // bits 28-30
    dt.irq    = ((low >> 31) & 0x1) != 0;   // bit     31
    dt.addr   = (low >> 32) & 0x7FFFFFFF;   // bits 32-62 (we ignore bit 63: memory select)
    return dt;
}

void EE_DMAC::process_chain_mode(DMAC_Channel& channel) {
    // Continue transferring until a tag with "tag_end" is encountered.
    bool tag_end = false;
    while (!tag_end) {
        // First, transfer the current QWC quadwords (just as in burst mode).
        while (channel.QWC > 0) {
            uint128_t data = bus.read128(channel.MADR);
            // For channels that output to a peripheral, you might write to a FIFO here.
            // For our example, we assume the bus/gif interface can be used.
            if (!bus.gif.is_path3_masked()) {
                bus.gif.write_fifo(data, channel.MADR, channel.QWC, true);
            } else {
                Logger::warn("Chain mode PATH3 masked; ignoring FIFO write");
            }
            channel.MADR += 16;
            channel.QWC -= 1;
        }

        // Once the current block is transferred, read the next DMAtag from TADR.
        uint128_t tag_raw = bus.read128(channel.TADR);
        DMAtag tag = parse_dmatag(tag_raw);

        // For debugging:
        // Logger::info("Chain tag: QWC= " + std::to_string(tag.qwc) +
        //              ", tag_id= " + std::to_string(tag.tag_id) +
        //              ", addr= 0x" + format("{:08X}", tag.addr) +
        //              ", irq= " + std::to_string(tag.irq));

        // Update the QWC from the tag. (Note: When TTE is enabled, extra data would be transferred
        // before the QWC; here we ignore that possibility.)
        channel.QWC = tag.qwc;

        // Process the tag by its tag_id.
        // (The following is based on the “Source Chain Tag ID” descriptions.)
        switch (tag.tag_id) {
            case 0: // REFE: set MADR from tag and finish transfer.
                channel.MADR = tag.addr;
                channel.TADR += 16;
                tag_end = true;
                break;

            case 1: // CNT: set MADR = (TADR + 16) then update TADR to point to following tag.
                channel.MADR = channel.TADR + 16;
                channel.TADR = channel.MADR;
                break;

            case 2: // NEXT: set MADR from next word and update TADR from tag.
                channel.MADR = channel.TADR + 16;
                channel.TADR = tag.addr;
                break;

            case 3: // REF: set MADR directly from tag, advance TADR.
                channel.MADR = tag.addr;
                channel.TADR += 16;
                break;

            case 4: // REFS: same as REF.
                channel.MADR = tag.addr;
                channel.TADR += 16;
                break;

            case 5: { // CALL: push return address into the address stack and update TADR.
                // CHCR bits 4-5 hold the address stack pointer (ASP).
                uint32_t asp = (channel.CHCR >> 4) & 0x3;
                // Save the return address in ASR0 or ASR1.
                if (asp == 0)
                    channel.ASR0 = channel.TADR + 16 + channel.QWC * 16;
                else if (asp == 1)
                    channel.ASR1 = channel.TADR + 16 + channel.QWC * 16;
                else
                    Logger::warn("CALL tag with unexpected ASP value: " + std::to_string(asp));
                // Set MADR to the word after the tag.
                channel.MADR = channel.TADR + 16;
                // Update TADR to the tag’s address.
                channel.TADR = tag.addr;
                // Increment ASP (store back in CHCR bits 4-5).
                asp = (asp + 1) & 0x3;
                channel.CHCR = (channel.CHCR & ~(0x3 << 4)) | (asp << 4);
                break;
            }
            case 6: { // RET: pop the return address from the address stack and update TADR.
                uint32_t asp = (channel.CHCR >> 4) & 0x3;
                channel.MADR = channel.TADR + 16;
                if (asp == 2)
                    channel.TADR = channel.ASR1;
                else if (asp == 1)
                    channel.TADR = channel.ASR0;
                else {
                    tag_end = true;
                    break;
                }
                // Decrement ASP.
                if (asp > 0) asp--;
                channel.CHCR = (channel.CHCR & ~(0x3 << 4)) | (asp << 4);
                break;
            }
            case 7: // END: set MADR from TADR and finish transfer.
                channel.MADR = channel.TADR + 16;
                tag_end = true;
                break;

            default:
                Logger::error("Unknown DMAtag ID: " + std::to_string(tag.tag_id));
                Neo2::exit(1, Neo2::Subsystem::EE_DMAC);
                break;
        }
    }
}