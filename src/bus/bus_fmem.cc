#include <bus/bus.hh>
#include <bus/bus_fmem.hh>
#include <ee/sio/sio.hh>
#include <log/log.hh>
#include <cstring>
#include <cstdio>
#include <neo2.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

std::uint32_t MCH_DRD = 0;
std::uint32_t MCH_RICM = 0;
std::uint32_t rdram_sdevid = 0;

std::uint32_t dev9_delay3 = 0;

// SBUS
std::uint32_t sbus_reg40 = 0;
std::uint32_t sbus_reg60 = 0;
std::uint32_t sbus_msflag = 0;
std::uint32_t sbus_maddr = 0;
std::uint32_t sbus_saddr = 0;
std::uint32_t sbus_smflag = 0;

std::uint32_t iop_cache_reg = 0;

void Bus::fmem_init()
{
    tlb = TLB(42);

    // Initialize TLB entries for BIOS and RAM
    TLBEntry entry;

    // KSEG1 (uncached BIOS)
    entry.page_mask = 0x0FFF;
    entry.entry_hi = 0xBFC00000; // Virtual address
    entry.entry_lo0 = 0x1FC00000 >> 12; // Physical address shifted (PFN for 0x1FC00000)
    entry.entry_lo1 = 0x1FC01000 >> 12; // Next page
    entry.global = true;
    tlb.write_entry_(0, entry);

    // KSEG0 (cached BIOS)
    entry.entry_hi = 0x9FC00000;
    entry.entry_lo0 = 0x1FC00000 >> 12; // Physical address shifted (PFN for 0x1FC00000)
    entry.entry_lo1 = 0x1FC01000 >> 12; // Next page
    tlb.write_entry_(1, entry);

    // KUSEG (RAM)
    entry.page_mask = 0x01FF;
    entry.entry_hi = 0x00000000;
    entry.entry_lo0 = 0x00000000 >> 12; // Physical address shifted (PFN for 0x00000000)
    entry.entry_lo1 = 0x00400000 >> 12; // For next 4MB page
    entry.global = true;
    tlb.write_entry_(2, entry);
}

template <typename T>
T io_read(Bus *bus, std::uint32_t address) {
    switch (address) {
        case 0x10000000 ... 0x10001830:
        if constexpr (sizeof(T) != 16) {
            return static_cast<T>(bus->timers.read(address));
        }
        break;

        case 0x10002000 ... 0x10002030:
        if constexpr (sizeof(T) != 16) {
            return static_cast<T>(bus->ipu.read(address));
        }
        break;

        case 0x10003000 ... 0x100030A0:
            if constexpr (sizeof(T) != 16) {
                return static_cast<T>(bus->gif.read(address));
            }
            break;

        case 0x10003800 ... 0x10005FFF:
            if constexpr (sizeof(T) != 16) {
                return static_cast<T>(bus->vif.read(address));
            }
            break;   

        case 0x10006000 ... 0x1000600C:
            if constexpr (sizeof(T) != 16) {
                return static_cast<T>(bus->gif.read(address));
            }
            break;


        case 0x10007000 ... 0x10007020:
            if constexpr (sizeof(T) != 16) {
                return static_cast<T>(bus->ipu.read(address));
            }
            break;
        
        case 0x10008000 ... 0x1000D4FF:
        {
            if constexpr (sizeof(T) != 16) {
                return static_cast<T>(bus->dmac.read_register(address));
            }
            break;
        }

        case 0x1000E000 ... 0x1000E050:
        {
            if constexpr (sizeof(T) != 16) {
                return static_cast<T>(bus->dmac.read_register(address));
            }
            break;
        }

        case 0x1000F000:
        {
            if constexpr (sizeof(T) != 16) {
                return static_cast<uint32_t>(bus->ee_intc.read(address));
            }
            break;
        }

        case 0x1000F010:
        {
            if constexpr (sizeof(T) != 16) {
                return static_cast<uint32_t>(bus->ee_intc.read(address));
            }
            break;
        }

        case 0x1000F100:
        case 0x1000F110:
        case 0x1000F120:
        case 0x1000F130:
        case 0x1000F140:
        case 0x1000F150:
        case 0x1000F180:
        case 0x1000F1C0:
        {
            if constexpr (sizeof(T) != 16) {
                return static_cast<uint32_t>(bus->sio.read(address));
            }
            break;
        }
        
        // SIF_MSCOM
        case 0x1000F200: {
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<uint32_t>(bus->sif.read(address));
            }
            break;
        }

        // SIF_SMCOM
        case 0x1000F210: {
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<uint32_t>(bus->sif.read(address));
            }
            break;
        }

        // SIF_MSFLG
        case 0x1000F220: {
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<uint32_t>(bus->sif.read(address));
            }
            break;
        }

        // SIF_SMFLG
        case 0x1000F230: {
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<uint32_t>(bus->sif.read(address));
            }
            break;
        }

        case 0x1000F400:
        case 0x1000F410:
        case 0x1000F420:
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<T>(0x0);
            }
            break;

        case 0x1000F430:
        case 0x1000F440:
            if constexpr (sizeof(T) == 4)
            {
                return bus->rdram.read(address);
            }
            else
            {
                Logger::error("Unhandled non-32-bit read to RDRAM registers");
                Neo2::exit(1, Neo2::Subsystem::Bus);
            }
            break;

        case 0x1000F450:
        case 0x1000F460:
        case 0x1000F480:
            return static_cast<T>(0x0);

        case 0x1000F510 ... 0x1000F590:
        {
            if constexpr (sizeof(T) != 16) {
                return static_cast<T>(bus->dmac.read_register(address));
            }
            break;
        }

        case 0x12000000 ... 0x12001080:
            if constexpr (sizeof(T) != 16) {
                return static_cast<T>(bus->gs.read(address));
            }
            break;

        // ?
        case 0x1A000006:
        {
            if constexpr (sizeof(T) == 2) {
                static int state = 0;

                if (state == 2) {
                    return static_cast<T>(0xAABB);
                } else {
                    if (state == 1) {
                        state = 2;
                    } else {
                        state = 1;
                    }
                    return static_cast<T>(0x0);
                }
            }
            break;
        }

        case 0x1A000010:
        {
            if constexpr (sizeof(T) == 2) {
                return static_cast<T>(0x0);
            }
            break;
        }

        // CDVD
        case 0x1F402004 ... 0x1F402008:
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<T>(bus->cdvd.read(address));
            }
            break;

        // CDVD
        case 0x1F40200A:
        case 0x1F40200B:
        case 0x1F40200F:
        case 0x1F402016:
        case 0x1F402017:
        case 0x1F402018:
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<T>(bus->cdvd.read(address));
            }
            break;

        // EE RDRAM initialization
        case 0x1F801010:
            return static_cast<T>(0x0);

        // DEV9?
        case 0x1F80146E:
        {
            if constexpr (sizeof(T) == 1)
            {
                return static_cast<T>(0x0);
            }
            break;
        }

        case 0x1F801100 ... 0x1F80112F: {
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<T>(bus->iop_timers.read(address));
            }
            break;
        }

        case 0x1F80141C:
        {
            if constexpr (sizeof(T) == 4) {
                return static_cast<uint32_t>(dev9_delay3);
            }
            break;
        }
        
        // IOP DMAC
        case 0x1F801578:
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<uint32_t>(bus->iop_dmac.read(address));
            }
            break;

        // TOOL Model?
        /*
            https://web.archive.org/web/20210321075609/
            https://www.obscuregamers.com/threads/running-ps1-game-on-dtl-t10000-tool.1949/
        */
        case 0x1F803204:
        {
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<T>(0x20);
            }
            break;
        }

        case 0x1F803800:
        {
            if constexpr (sizeof(T) == 2) {
                return static_cast<uint16_t>(0);
            }
            break;
        }

        case 0x1F801450: {
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<T>(0x0);
            }
            break;
        }

        case 0x1F801480 ... 0x1F8014AF: {
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<T>(bus->iop_timers.read(address));
            }
            break;
        }

        case 0x1F808200 ... 0x1F80823F:
        case 0x1F808240 ... 0x1F80825F:
        case 0x1F808260:
        case 0x1F808264:
        case 0x1F808268:
        case 0x1F80826C:
        case 0x1F808270:
        case 0x1F808274:
        case 0x1F808280:
            if constexpr (sizeof(T) != 16) {
                return static_cast<T>(bus->sio2.read(address));
            }
            break;

        case 0x1F808400:
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<T>(bus->firewire.read(address));
            }
            break;

        // TODO: Use firewire.read
        case 0x1F80847C:
            if constexpr (sizeof(T) == 4)
            {
                return static_cast<uint32_t>(0x10000001);
            }
            else
            {
                Logger::error("Unhandled non-32-bit read to 0x1F80847C register");
                return static_cast<T>(Neo2::exit(1, Neo2::Subsystem::Bus));
            }
            break;

        case 0xFFFE0130: {
            if constexpr (sizeof(T) == 4)
            {
                return static_cast<T>(iop_cache_reg);
            }
            break;
        }
        case 0xFFFFFFF0 ... 0xFFFFFFFF: {
            if constexpr (sizeof(T) != 16)
            {
                return static_cast<T>(0x0);
            }
            break;
        }
        default:
            goto ret;
    }

ret:
    std::string type_str = (sizeof(T) == 1   ? "8-bit read"
                            : sizeof(T) == 2 ? "16-bit read"
                            : sizeof(T) == 4 ? "32-bit read"
                            : sizeof(T) == 8 ? "64-bit read"
                                             : "128-bit read");
    std::string msg = type_str + " from unknown address: 0x" + format("{:08X}", address);
    Logger::error(msg.c_str());
    return static_cast<T>(Neo2::exit(1, Neo2::Subsystem::Bus));
}

template <typename T>
void io_write(Bus *bus, std::uint32_t address, T value) {
    switch (address) {
        case 0x10000000 ... 0x10001830:
        if constexpr (sizeof(T) != 16) {
            bus->timers.write(address, value);
        }
        break;

        case 0x10002000 ... 0x10002030:
        if constexpr (sizeof(T) != 16) {
            bus->ipu.write(address, value);
        }
        break;

        case 0x10003000 ... 0x100030A0:
            if constexpr (sizeof(T) != 16) {
                bus->gif.write(address, value);
            }
            break;

        case 0x10003800 ... 0x10005FFF:
            if constexpr (sizeof(T) != 16) {
                bus->vif.write(address, value);
            }
            break;   

        case 0x10006000 ... 0x1000600C:
            if constexpr (sizeof(T) != 16) {
                bus->gif.write(address, value);
            }
            break;

        case 0x10007000 ... 0x10007020:
            if constexpr (sizeof(T) != 16) {
                bus->ipu.write(address, value);
            }
            break;
        
        case 0x10008000 ... 0x1000D4FF:
        {
            if constexpr (sizeof(T) != 16) {
                bus->dmac.write_register(address, value);
            }
            break;
        }

        case 0x1000E000 ... 0x1000E050:
        {
            if constexpr (sizeof(T) != 16) {
                bus->dmac.write_register(address, value);
            }
            break;
        }

        case 0x1000F000:
        {
            if constexpr (sizeof(T) == 4) {
                bus->ee_intc.write(address, value);
            }
            break;
        }

        case 0x1000F010:
        {
            if constexpr (sizeof(T) == 4) {
                bus->ee_intc.write(address, value);
            }
            break;
        }

        case 0x1000F100:
        case 0x1000F110:
        case 0x1000F120:
        case 0x1000F130:
        case 0x1000F140:
        case 0x1000F150:
        case 0x1000F180:
        case 0x1000F1C0:
        {
            if constexpr (sizeof(T) != 16) {
                bus->sio.write(address, static_cast<uint32_t>(value));
            }
            break;
        }

        // SIF_MSCOM
        case 0x1000F200: {
            if constexpr (sizeof(T) != 16)
            {
                bus->sif.write(address, value);
            }
            break;
        }

        // SIF_SMCOM
        case 0x1000F210: {
            if constexpr (sizeof(T) != 16)
            {
                bus->sif.write(address, value);
            }
            break;
        }

        // SIF_MSFLG
        case 0x1000F220: {
            if constexpr (sizeof(T) != 16)
            {
                bus->sif.write(address, value);
            }
            break;
        }

        // SIF_SMFLG
        case 0x1000F230: {
            if constexpr (sizeof(T) != 16)
            {
                bus->sif.write(address, value);
            }
            break;
        }

        // SIF_CTRL
        case 0x1000F240: {
            if constexpr (sizeof(T) != 16)
            {
                bus->sif.write(address, value);
            }
            break;
        }

        case 0x1000F260:
            break;

        case 0x1000F400:
        case 0x1000F410:
        case 0x1000F420:
            break;

        case 0x1000F430:
        case 0x1000F440:
            if constexpr (sizeof(T) == 4)
            {
                bus->rdram.write(address, value);
            }
            else
            {
                Logger::error("Unhandled non-32-bit write to RDRAM registers");
                Neo2::exit(1, Neo2::Subsystem::Bus);
            }
            break;

        case 0x1000F450:
        case 0x1000F460:
        case 0x1000F480:
        case 0x1000F490:
        case 0x1000F4A0:
            break;

        case 0x1000F500:
            break;

        case 0x1000F510 ... 0x1000F590:
        {
            if constexpr (sizeof(T) != 16) {
                bus->dmac.write_register(address, value);
            }
            break;
        }

        // VU1 Data Memory
        case 0x1100C000 ... 0x1100FFFF: {
            break;
        }

        // VU0 Code Memory
        case 0x11000000 ... 0x11000FFF: {
            break;
        }

        // VU0 Data Memory
        case 0x11004000 ... 0x11004FFF: {
            break;
        }

        // VU1 Code Memory
        case 0x11008000 ... 0x1100BFFF: {
            break;
        }

        case 0x12000000 ... 0x12001080:
            if constexpr (sizeof(T) != 16) {
                bus->gs.write(address, value);
            }
            break;

        // IOP?
        case 0x1A000000:
        case 0x1A000002:
        case 0x1A000004:
        case 0x1A000006:
        case 0x1A000008:
        case 0x1A000010:
        case 0x1A000012:
        {
            if constexpr (sizeof(T) != 16) {
                return;
            }
            break;
        }
        
        // CDVD
        case 0x1F402004 ... 0x1F402008:
            if constexpr (sizeof(T) != 16)
            {
                bus->cdvd.write(address, value);
            }
            break;

        // CDVD
        case 0x1F40200A:
        case 0x1F40200B:
        case 0x1F40200F:
        case 0x1F402016:
        case 0x1F402017:
        case 0x1F402018:
            if constexpr (sizeof(T) != 16)
            {
                bus->cdvd.write(address, value);
            }
            break;

        // Expansion 1 Base Address
        case 0x1F801000:
            break;

        // Expansion 2 Base Address
        case 0x1F801004:
            break;

        // Expansion 1 Delay Register?
        case 0x1F801008:
            break;

        // Expansion 3 Delay Register?
        case 0x1F80100C:
            break;

        // SSBUS Delay Register?
        case 0x1F801010:
            break;

        // SPU Delay Register?
        case 0x1F801014:
            break;

        // CDROM Delay Register?
        case 0x1F801018:
            break;

        // ? Delay Register?
        case 0x1F80101C:
            break;

        // ? Register
        case 0x1F801020:
            break;

        // IOP RAM SIZE Register
        case 0x1F801060:
            break;

        // IOP DPCR - DMA Priority/Enable
        case 0x1F8010F0:
            if constexpr (sizeof(T) != 16)
            {
                bus->iop_dmac.write(address, value);
            }
            break;

        case 0x1F801100 ... 0x1F80112F: {
            if constexpr (sizeof(T) != 16)
            {
                bus->iop_timers.write(address, value);
            }
            break;
        }

        // IOP Boot Mode?
        case 0x1F801450: {
            break;
        }

        // SSBUS?
        case 0x1F801400:
        case 0x1F801404:
        case 0x1F801408:
        case 0x1F80140C:
        case 0x1F801410:
        case 0x1F801414:
        case 0x1F801418:
        case 0x1F801420:
            break;

        case 0x1F80141C:
        {
            if constexpr (sizeof(T) == 4) {
                dev9_delay3 = static_cast<uint32_t>(value);
            }
            break;
        }

        case 0x1F801470:
        case 0x1F801472:
            if constexpr (sizeof(T) == 2) {
                break;
            } else {
                goto error;
            }
            break;

        case 0x1F801480 ... 0x1F8014AF: {
            if constexpr (sizeof(T) != 16)
            {
                bus->iop_timers.write(address, value);
            }
            break;
        }

        // IOP DMAC
        case 0x1F801570:
        case 0x1F801578:
            if constexpr (sizeof(T) != 16)
            {
                bus->iop_dmac.write(address, value);
            }
            break;

        // PS2 IOP BIOS POST2
        case 0x1F802070: {
            if constexpr (sizeof(T) != 16)
            {
                Logger::debug("IOP POST2 BIOS: " + format("{:02X}", value & 0xFF));
            }
            break;
        }

        case 0x1F808200 ... 0x1F80823F:
        case 0x1F808240 ... 0x1F80825F:
        case 0x1F808260:
        case 0x1F808264:
        case 0x1F808268:
        case 0x1F80826C:
        case 0x1F808270:
        case 0x1F808274:
        case 0x1F808280:
            if constexpr (sizeof(T) != 16)
            {
                bus->sio2.write(address, value);
            }
            break;

        case 0x1F808414:
        case 0x1F808420:
        case 0x1F808428:
        case 0x1F808430:
        case 0x1F80844C:
            if constexpr (sizeof(T) != 16)
            {
                bus->firewire.write(address, value);
            }
            break;

        // IOP Cache Register
        case 0xFFFE0130:
            if constexpr (sizeof(T) != 16)
            {
                iop_cache_reg = static_cast<uint32_t>(value);
            }
            break;

        // ?
        case 0xFFFE0140:
        case 0xFFFE0144:
            break;

        case 0xFFFFFFF0 ... 0xFFFFFFFF: {
            break;
        }
        default:
error:
            std::string type_str = (sizeof(T) == 1 ? "8-bit write"  :
                                    sizeof(T) == 2 ? "16-bit write" :
                                    sizeof(T) == 4 ? "32-bit write" :
                                    sizeof(T) == 8 ? "64-bit write" : "128-bit write");
            std::string msg = type_str + " to unknown address: 0x" + format("{:08X}", address);
            Logger::error(msg.c_str());
            Neo2::exit(1, Neo2::Subsystem::Bus);
            break;
    }
}

std::uint8_t Bus::fmem_read8(std::uint32_t address)
{
    if (address >= 0x70000000 && address < 0x70004000) {
        return scratchpad[address - 0x70000000];
    }

    address = map_to_phys(address, tlb); // Map the address to physical address
    const auto page = address >> 12;            // Divide the address by 4KB to get the page number
    const auto offset = address & 0xFFF;        // The offset inside the 4KB page
    const auto pointer = address_space_r[page]; // Get the pointer to the page
    uint8_t result;

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
    {
        result = *(uint8_t *)(pointer + offset); // Read 8-bit value from the fast page
    }
    else
    {
        result = io_read<uint8_t>(this, address); // If it's not in the fast page, do an IO read
    }

    return result;
}

std::uint16_t Bus::fmem_read16(std::uint32_t address)
{
    if (address >= 0x70000000 && address < 0x70004000) {
        return *reinterpret_cast<std::uint16_t*>(&scratchpad[address - 0x70000000]);
    }

    address = map_to_phys(address, tlb);
    const auto page = address >> 12;            // Divide the address by 4KB to get the page number.
    const auto offset = address & 0xFFF;        // The offset inside the 4KB page
    const auto pointer = address_space_r[page]; // Get the pointer to this page
    uint16_t result;

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
    {
        result = *(uint16_t *)(pointer + offset);
    }
    else
    {
        result = io_read<uint16_t>(this, address);
    }

    return result;
}

std::uint32_t Bus::fmem_read32(std::uint32_t address)
{
    if (address >= 0x70000000 && address < 0x70004000) {
        return *reinterpret_cast<std::uint32_t*>(&scratchpad[address - 0x70000000]);
    }

    address = map_to_phys(address, tlb);
    const auto page = address >> 12;            // Divide the address by 4KB to get the page number.
    const auto offset = address & 0xFFF;        // The offset inside the 4KB page
    const auto pointer = address_space_r[page]; // Get the pointer to this page
    uint32_t result;

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
    {
        result = *(uint32_t *)(pointer + offset);
    }
    else if (address >= 0xBF000000 && address < 0xBFC00000)
    {
        // TODO: Minor hack for debugger not to exit
        result = 0x00000000;
    }
    else
    {
        result = io_read<uint32_t>(this, address);
    }

    return result;
}

std::uint64_t Bus::fmem_read64(std::uint32_t address)
{
    if (address >= 0x70000000 && address < 0x70004000) {
        return *reinterpret_cast<std::uint64_t*>(&scratchpad[address - 0x70000000]);
    }

    address = map_to_phys(address, tlb);

    const auto page = address >> 12;            // Divide the address by 4KB to get the page number.
    const auto offset = address & 0xFFF;        // The offset inside the 4KB page
    const auto pointer = address_space_r[page]; // Get the pointer to this page
    uint64_t result;

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
    {
        result = *(uint64_t *)(pointer + offset);
    }
    else
    {
        result = io_read<uint64_t>(this, address);
    }

    return result;
}

uint128_t Bus::fmem_read128(uint32_t address) {
    uint128_t result;

    if (address >= 0x70000000 && address < 0x70004000) {
        uint64_t lower = *reinterpret_cast<uint64_t*>(&scratchpad[address - 0x70000000]);
        uint64_t upper = *reinterpret_cast<uint64_t*>(&scratchpad[address - 0x70000000 + 8]);
        result.u64[0] = lower;
        result.u64[1] = upper;
        return result;
    }

    address = map_to_phys(address, tlb);
    const auto page = address >> 12;
    const auto offset = address & 0xFFF;
    const auto pointer = address_space_r[page];

    if (pointer != 0) {
        result.u64[0] = *(uint64_t *)(pointer + offset);
        result.u64[1] = *(uint64_t *)(pointer + offset + 8);
    } else if (address >= 0xBF000000 && address < 0xBFC00000) {
        result.u128 = 0;
    } else {
        result = io_read<uint128_t>(this, address);
    }
    return result;
}

void Bus::fmem_write8(std::uint32_t address, std::uint8_t value)
{
    if (address >= 0x70000000 && address < 0x70004000) {
        scratchpad[address - 0x70000000] = value;
        return;
    }

    address = map_to_phys(address, tlb);
    const auto page = address >> 12;            // Divide the address by 4KB to get the page number.
    const auto offset = address & 0xFFF;        // The offset inside the 4KB page
    const auto pointer = address_space_w[page]; // Get the pointer to this page

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
    {
        *(uint8_t *)(pointer + offset) = value;
    }
    else
    {
        io_write<uint8_t>(this, address, value);
    }
}

void Bus::fmem_write16(std::uint32_t address, std::uint16_t value)
{
    if (address >= 0x70000000 && address < 0x70004000) {
        *reinterpret_cast<std::uint16_t*>(&scratchpad[address - 0x70000000]) = value;
        return;
    }

    address = map_to_phys(address, tlb);
    const auto page = address >> 12;            // Divide the address by 4KB to get the page number.
    const auto offset = address & 0xFFF;        // The offset inside the 4KB page
    const auto pointer = address_space_w[page]; // Get the pointer to this page

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
    {
        *(uint16_t *)(pointer + offset) = value;
    }
    else
    {
        io_write<uint16_t>(this, address, value);
    }
}

void Bus::fmem_write32(std::uint32_t address, std::uint32_t value)
{
    if (address >= 0x70000000 && address < 0x70004000) {
        *reinterpret_cast<std::uint32_t*>(&scratchpad[address - 0x70000000]) = value;
        return;
    }

    address = map_to_phys(address, tlb);
    const auto page = address >> 12;            // Divide the address by 4KB to get the page number.
    const auto offset = address & 0xFFF;        // The offset inside the 4KB page
    const auto pointer = address_space_w[page]; // Get the pointer to this page

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
    {
        *(uint32_t *)(pointer + offset) = value;
    }
    else
    {
        io_write<uint32_t>(this, address, value);
    }
}

void Bus::fmem_write64(std::uint32_t address, std::uint64_t value)
{
    if (address >= 0x70000000 && address < 0x70004000) {
        *reinterpret_cast<std::uint64_t*>(&scratchpad[address - 0x70000000]) = value;
        return;
    }

    address = map_to_phys(address, tlb);  // Map the virtual address to a physical address
    const auto page = address >> 12;                        // Divide the address by 4KB to get the page number
    const auto offset = address & 0xFFF;                    // The offset inside the 4KB page
    const auto pointer = address_space_w[page];             // Get the pointer to this page

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
    {
        *(uint64_t *)(pointer + offset) = value;  // Store the 64-bit value at the effective address
    }
    else
    {
        io_write<uint64_t>(this, address, value);
    }
}

void Bus::fmem_write128(uint32_t address, uint128_t value) {
    if (address >= 0x70000000 && address < 0x70004000) {
        // Split the 128-bit value into two 64-bit values
        uint64_t lower = value.u64[0];
        uint64_t upper = value.u64[1];
        *reinterpret_cast<uint64_t*>(&scratchpad[address - 0x70000000]) = lower;
        *reinterpret_cast<uint64_t*>(&scratchpad[address - 0x70000000 + 8]) = upper;
        return;
    }

    address = map_to_phys(address, tlb);  // Map virtual address to physical address
    const auto page = address >> 12;                        // Calculate page (address >> 12 gives the page number)
    const auto offset = address & 0xFFF;                    // Calculate offset within the page (address & 0xFFF gives the offset)
    const auto pointer = address_space_r[page];             // Get the pointer to the page in memory

    if (pointer != 0) {
        // Write the 128-bit value (split into two 64-bit parts) into memory at the correct offset
        *(uint64_t *)(pointer + offset) = value.u64[0];      // Write the lower 64-bits
        *(uint64_t *)(pointer + offset + 8) = value.u64[1];  // Write the upper 64-bits
    } else if (address >= 0xBF000000 && address < 0xBFC00000) {
        // Special case: Address is in the reserved range, no action needed
        // Since it's read-only, we don't write anything here.
    } else {
        io_write<uint128_t>(this, address, value);
    }
}

void Bus::fmem_write32_dbg(std::uint32_t address, std::uint32_t value, std::uint32_t pc)
{
    Logger::info("SW DBG: Address -> 0x" + format("{:08X}", address) + " , value: 0x" 
    + format("{:08X}", value) + " , PC: 0x" + format("{:08X}", pc));
    fmem_write32(address, value);
}
