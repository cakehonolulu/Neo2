#include <iop/iop_dmac.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif
#include <neo2.hh>

IOP_DMAC::IOP_DMAC() {
    Logger::set_subsystem("IOP_DMAC");
    for (int i = 0; i < 13; ++i) {
        channels[i].madr = 0;
        channels[i].bcr = 0;
        channels[i].chcr = 0;
        channels[i].tadr = 0;
    }
    dpcr = 0x07777777; // Initial value on PS2 reset
    dpcr2 = 0;
    dicr = 0;
    dicr2 = 0;
    dmace = 0;
    dmacinten = 0;
}

uint32_t IOP_DMAC::read(uint32_t address) {
    std::string reg_name;
    uint32_t channel_index = (address >> 4) & 0xF; // Determine which channel

    switch (address & 0xFFF) {
        case 0x000:
            reg_name = format("DMA_MADR[{}]", channel_index);
            Logger::info("IOP DMA register read from " + reg_name);
            return channels[channel_index].madr;

        case 0x004:
            reg_name = format("DMA_BCR[{}]", channel_index);
            Logger::info("IOP DMA register read from " + reg_name);
            return channels[channel_index].bcr;

        case 0x008:
            reg_name = format("DMA_CHCR[{}]", channel_index);
            Logger::info("IOP DMA register read from " + reg_name);
            return channels[channel_index].chcr;

        case 0x00C:
            reg_name = format("DMA_TADR[{}]", channel_index);
            Logger::info("IOP DMA register read from " + reg_name);
            return channels[channel_index].tadr;

        case 0x0F0:
            reg_name = "DPCR";
            Logger::info("IOP DMA register read from " + reg_name);
            return dpcr;

        case 0x570:
            reg_name = "DPCR2";
            Logger::info("IOP DMA register read from " + reg_name);
            return dpcr2;

        case 0x0F4:
            reg_name = "DICR";
            Logger::info("IOP DMA register read from " + reg_name);
            return dicr;

        default:
            Logger::error("Invalid IOP DMA register write at address 0x" + format("{:08X}", address));
            break;
    }

    return Neo2::exit(1, Neo2::Subsystem::IOP);
}

void IOP_DMAC::write(uint32_t address, uint32_t value)
{
    std::string reg_name;
    uint32_t channel_index = (address >> 4) & 0xF; // Determine which channel

    switch (address & 0xFFF)
    {
    case 0x000:
        reg_name = format("DMA_MADR[{}]", channel_index);
        Logger::info("IOP DMA register write to " + reg_name);
        channels[channel_index].madr = value;
        break;

    case 0x004:
        reg_name = format("DMA_BCR[{}]", channel_index);
        Logger::info("IOP DMA register write to " + reg_name);
        channels[channel_index].bcr = value;
        break;

    case 0x008:
        reg_name = format("DMA_CHCR[{}]", channel_index);
        Logger::info("IOP DMA register write to " + reg_name);
        channels[channel_index].chcr = value;
        break;

    case 0x00C:
        reg_name = format("DMA_TADR[{}]", channel_index);
        Logger::info("IOP DMA register write to " + reg_name);
        channels[channel_index].tadr = value;
        break;

    case 0x0F0:
        reg_name = "DPCR";
        Logger::info("IOP DMA register write to " + reg_name);
        dpcr = value;
        break;

    case 0x570:
        reg_name = "DPCR2";
        Logger::info("IOP DMA register write to " + reg_name);
        dpcr2 = value;
        break;

    case 0x0F4:
        reg_name = "DICR";
        Logger::info("IOP DMA register write to " + reg_name);
        dicr = value;
        break;

    case 0x574:
        reg_name = "DICR2";
        Logger::info("IOP DMA register write to " + reg_name);
        dicr2 = value;
        break;

    case 0x578:
        reg_name = "DMACEN";
        Logger::info("IOP DMA register write to " + reg_name);
        dmace = value;
        break;

    case 0x57C:
        reg_name = "DMACINTEN";
        Logger::info("IOP DMA register write to " + reg_name);
        dmacinten = value;
        break;

    default:
        Logger::error("Invalid IOP DMA register write at address 0x" + format("{:08X}", address));
        break;
    }
}
