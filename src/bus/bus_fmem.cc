#include <bus/bus.hh>
#include <bus/bus_fmem.hh>
#include <ee/sio/sio.hh>
#include <log/log.hh>
#include <cstring>
#include <execinfo.h>
#include <cxxabi.h>
#include <unistd.h>
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

void Bus::fmem_init() {
    tlb = TLB(42);

    // Initialize TLB entries for BIOS and RAM
    TLBEntry entry;

    // KSEG1 (uncached BIOS)
    entry.page_mask = 0x1FFF; // 4KB pages
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
    entry.page_mask = 0x007FE000; // 4MB pages
    entry.entry_hi = 0x00000000;
    entry.entry_lo0 = 0x00000000 >> 12; // Physical address shifted (PFN for 0x00000000)
    entry.entry_lo1 = 0x00400000 >> 12; // For next 4MB page
    entry.global = true;
    tlb.write_entry_(2, entry);
    // KSEG1 (uncached BIOS, 4KB pages)
    /*entry.page_mask = 0x00001FFF; // 4KB page size (mask)
    entry.vpn2 = 0xBFC00000 >> 13; // VPN2 (Virtual Page Number / 2)
    entry.pfn0 = 0x1FC00000 >> 12; // PFN0 (Physical Frame Number for even page)
    entry.pfn1 = 0x1FC01000 >> 12; // PFN1 (Physical Frame Number for odd page)
    entry.v0 = true;               // Even page valid
    entry.v1 = true;               // Odd page valid
    entry.d0 = true;               // Even page writable (dirty)
    entry.d1 = true;               // Odd page writable (dirty)
    entry.c0 = 2;                  // Cache mode for even page: Uncached
    entry.c1 = 2;                  // Cache mode for odd page: Uncached
    entry.asid = 0;                // Address Space ID (not relevant here)
    entry.global = true;           // Global mapping
    tlb.write_entry_(0, entry);

    // KSEG0 (cached BIOS, 4KB pages)
    entry.vpn2 = 0x9FC00000 >> 13; // VPN2 (Virtual Page Number / 2)
    entry.c0 = 3;                  // Cache mode for even page: Cached
    entry.c1 = 3;                  // Cache mode for odd page: Cached
    tlb.write_entry_(1, entry);

    // KUSEG (RAM, 4MB pages)
    entry.page_mask = 0x007FE000;  // 4MB page size (mask)
    entry.vpn2 = 0x00000000 >> 13; // VPN2 (Virtual Page Number / 2)
    entry.pfn0 = 0x00000000 >> 12; // PFN0 (Physical Frame Number for even page)
    entry.pfn1 = 0x00400000 >> 12; // PFN1 (Physical Frame Number for odd page)
    entry.v0 = true;               // Even page valid
    entry.v1 = true;               // Odd page valid
    entry.d0 = true;               // Even page writable (dirty)
    entry.d1 = true;               // Odd page writable (dirty)
    entry.c0 = 3;                  // Cache mode for even page: Cached
    entry.c1 = 3;                  // Cache mode for odd page: Cached
    entry.global = true;           // Global mapping
    tlb.write_entry_(2, entry);*/

    // PS2's Address Space is 4GB, divided into 4KB pages
    address_space_r = new uintptr_t[0x100000];  // Readable memory pages (4KB * 1024 * 1024 = 4GB)
    address_space_w = new uintptr_t[0x100000];  // Writable memory pages

    memset(address_space_r, 0, sizeof(uintptr_t) * 0x100000);
    memset(address_space_w, 0, sizeof(uintptr_t) * 0x100000);

    // Map BIOS pages into KUSEG, KSEG0, and KSEG1 (uncached and cached)
    for (auto pageIndex = 0; pageIndex < 1024; pageIndex++) {
        const auto pointer = (uintptr_t)&bios[(pageIndex * PAGE_SIZE)]; // pointer to page #pageIndex of the BIOS

        // Map BIOS to KUSEG, KSEG0, KSEG1
        address_space_r[pageIndex + 0x1FC00] = pointer;  // KUSEG BIOS
        address_space_r[pageIndex + 0x9FC00] = pointer;  // KSEG0 BIOS (cached)
        address_space_r[pageIndex + 0xBFC00] = pointer;  // KSEG1 BIOS (uncached)
    }

    // Main RAM: 32 MB starting at physical address 0x00000000
    for (auto pageIndex = 0; pageIndex < 0x8000; pageIndex++) {  // 32 MB / 4KB per page = 0x8000 pages
        const auto pointer = (uintptr_t)&ram[(pageIndex * PAGE_SIZE)];  // Main RAM pointer
        address_space_r[pageIndex] = pointer;  // Map to KUSEG main RAM
        address_space_w[pageIndex] = pointer;  // Map to KSEG0/KSEG1 main RAM (can write to it)
    }
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
        
        // EE RDRAM initialization
        case 0x1F801010:
            return static_cast<T>(0x0);

        case 0x1000F200:
            return static_cast<T>(sbus_maddr);
            break;

        case 0x1000F210:
            return static_cast<T>(sbus_saddr);
            break;

        case 0x1000F220:
            return static_cast<T>(sbus_msflag);
            break;

        case 0x1000F230:
            return static_cast<T>(0x1a12301);
            break;

        case 0x1000F400:
        case 0x1000F410:
        case 0x1000F420:
        case 0x1000F430:
            return static_cast<T>(0x0);

        case 0x1000F440:
        {
            if constexpr (sizeof(T) == 4) {
                uint8_t SOP = (MCH_RICM >> 6) & 0xF;
                uint8_t SA = (MCH_RICM >> 16) & 0xFFF;
                if (!SOP)
                {
                    switch (SA)
                    {
                    case 0x21:
                        if (rdram_sdevid < 2)
                        {
                            rdram_sdevid++;
                            return 0x1F;
                        }
                        return 0;
                    case 0x23:
                        return 0x0D0D;
                    case 0x24:
                        return 0x0090;
                    case 0x40:
                        return MCH_RICM & 0x1F;
                    }
                }
                return 0;
            }
        }

        case 0x1000F450:
        case 0x1000F460:
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

        // Has to return 0 else starts a write loop from 0x1C000015 onwards?
        case 0x1C0003C0:
        {
            if constexpr (sizeof(T) == 4) {
                return static_cast<uint32_t>(0x0);
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

        case 0x1F803800:
        {
            if constexpr (sizeof(T) == 2) {
                return static_cast<uint16_t>(0);
            }
            break;
        }

        default:
            std::string type_str = (sizeof(T) == 1 ? "8-bit read"  :
                                    sizeof(T) == 2 ? "16-bit read" :
                                    sizeof(T) == 4 ? "32-bit read" :
                                    sizeof(T) == 8 ? "64-bit read" : "128-bit read");
            std::string msg = type_str + " from unknown address: 0x" + format("{:08X}", address);
            Logger::error(msg.c_str());
            return static_cast<T>(Neo2::exit(1, Neo2::Subsystem::Bus));
    }
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

        case 0x1000F400:
        case 0x1000F410:
        case 0x1000F420:
            break;

        case 0x1000F200:
            if constexpr (sizeof(T) == 4) {
                sbus_maddr = value;
            }
            break;

        case 0x1000F210:
            if constexpr (sizeof(T) == 4) {
                sbus_saddr = value;
            }
            break;

        case 0x1000F220:
            if constexpr (sizeof(T) == 4) {
                sbus_msflag = value;
            }
            break;

        case 0x1000F230:
            if constexpr (sizeof(T) == 4) {
                sbus_smflag = value;
            }
            break;

        case 0x1000F240:
        case 0x1000F260:
            break;

        case 0x1000F430:
        {
            if constexpr (sizeof(T) == 4) {
                uint8_t SA = (value >> 16) & 0xFFF;
                uint8_t SBC = (value >> 6) & 0xF;

                if (SA == 0x21 && SBC == 0x1 && ((MCH_DRD >> 7) & 1) == 0) rdram_sdevid = 0;

                MCH_RICM = value & ~0x80000000;
            }
            break;
        }

        case 0x1000F440:
            if constexpr (sizeof(T) == 4) {
                MCH_DRD = value;
            }
            break;

        case 0x1000F450:
        case 0x1000F460:
        case 0x1000F480:
        case 0x1000F490:
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

        // ?
        case 0x1A000008:
        {
            if constexpr (sizeof(T) == 2) {
                return;
            }
            break;
        }

        case 0x1C0003E0:
            break;

        case 0x1F801010:
            // NOP, unknown register
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
