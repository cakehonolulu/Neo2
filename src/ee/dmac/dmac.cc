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

EE_DMAC::EE_DMAC() {
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

    // Initialize channels based on their addresses
    channels[0x10008000] = DMAC_Channel(); // VIF0
    channels[0x10009000] = DMAC_Channel(); // VIF1
    channels[0x1000A000] = DMAC_Channel(); // GIF (PATH3)
    channels[0x1000B000] = DMAC_Channel(); // IPU_FROM
    channels[0x1000B400] = DMAC_Channel(); // IPU_TO
    channels[0x1000C000] = DMAC_Channel(); // SIF0 (from IOP)
    channels[0x1000C400] = DMAC_Channel(); // SIF1 (to IOP)
    channels[0x1000C800] = DMAC_Channel(); // SIF2 (bidirectional)
    channels[0x1000D000] = DMAC_Channel(); // SPR_FROM
    channels[0x1000D400] = DMAC_Channel(); // SPR_TO
}

void EE_DMAC::write_register(uint32_t address, uint32_t value) {
    uint32_t base_address = address & 0xFFFFF000;
    DMAC_Channel* channel = get_channel(base_address);

    if (channel) {
        switch (address & 0xFF) {
            case 0x00: channel->CHCR = value; break;
            case 0x10: channel->MADR = value; break;
            case 0x20: channel->QWC = value; break;
            case 0x30: channel->TADR = value; break;
            case 0x40: channel->ASR0 = value; break;
            case 0x50: channel->ASR1 = value; break;
            case 0x80: channel->SADR = value; break;
            default:
                Logger::error("Invalid channel register address: 0x" + format("{:08X}", address));
                break;
        }
    } else if (!channel) {
        switch (address) {
            case 0x1000E000: D_CTRL = value; break;
            case 0x1000E010: D_STAT = value; break;
            case 0x1000E020: D_PCR = value; break;
            case 0x1000E030: D_SQWC = value; break;
            case 0x1000E040: D_RBSR = value; break;
            case 0x1000E050: D_RBOR = value; break;
            case 0x1000F590: D_ENABLEW = value; break;
            default:
                Logger::error("Invalid DMAC address: 0x" + format("{:08X}", address));
                break;
        }
    }
}

uint32_t EE_DMAC::read_register(uint32_t address) {
    uint32_t base_address = address & 0xFFFFF000;
    DMAC_Channel* channel = get_channel(base_address);

    if (channel) {
        switch (address & 0xFF) {
            case 0x00: return channel->CHCR;
            case 0x10: return channel->MADR;
            case 0x20: return channel->QWC;
            case 0x30: return channel->TADR;
            case 0x40: return channel->ASR0;
            case 0x50: return channel->ASR1;
            case 0x80: return channel->SADR;
            default:
                    Logger::error("Invalid channel register address: 0x" + format("{:08X}", address));
                    break;
        }
    } else if (!channel) {
        switch (address) {
            case 0x1000E000: return D_CTRL;
            case 0x1000E010: return D_STAT;
            case 0x1000E020: return D_PCR;
            case 0x1000E030: return D_SQWC;
            case 0x1000E040: return D_RBSR;
            case 0x1000E050: return D_RBOR;
            case 0x1000F590: return D_ENABLEW;
            default:
                Logger::error("Invalid DMAC address: 0x" + format("{:08X}", address));
                break;
        }
    }
}

DMAC_Channel* EE_DMAC::get_channel(uint32_t address) {
    uint32_t base_address = address & 0xFFFFF000;
    auto it = channels.find(base_address);
    if (it != channels.end()) {
        return &it->second;
    }

    return nullptr;
}
