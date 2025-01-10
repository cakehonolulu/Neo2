#include <bus/bus.hh>
#include <bus/bus_fmem.hh>
#include <log/log.hh>
#include <cstring>
#include <execinfo.h>
#include <cxxabi.h>
#include <unistd.h>
#include <cstdio>

#if __has_include(<format>)
#include "neo2.hh"
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

void Bus::fmem_init()
{
    // PS2's Address Space is 4GB, divide it in 4KB pages
    address_space_r = new uintptr_t[0x100000];  // Readable memory pages (4KB * 1024 * 1024 = 4GB)
    address_space_w = new uintptr_t[0x100000];  // Writable memory pages

    const uint32_t PAGE_SIZE = 4 * 1024; // Page size = 4KB

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

std::uint32_t map_to_phys(std::uint32_t addr) {
    if (addr >= 0x80000000 && addr < 0xA0000000) {
        // KSEG0: Directly mapped, no translation needed
        return addr & 0x1FFFFFFF;  // Physical address (clearing the high 3 bits)
    }
    else if (addr >= 0xA0000000 && addr < 0xC0000000) {
        // KSEG1: Directly mapped, no translation needed
        return addr & 0x1FFFFFFF;  // Physical address (uncached)
    }
    else if (addr >= 0xBFC00000 && addr < 0xC0000000) {
        // BIOS or special addresses
        return addr & 0x1FFFFFFF;
    }
    else {
        std::string msg;
        msg = "Unhandled address translation for: 0x" + format("{:08X}", addr);
        // Handle other address regions or unknown addresses
        Logger::error(msg.c_str());
        return 0;
    }
}

std::uint32_t Bus::fmem_read32(std::uint32_t address)
{
    address = map_to_phys(address);
    const auto page = address >> 12;            // Divide the address by 4KB to get the page number.
    const auto offset = address & 0xFFF;        // The offset inside the 4KB page
    const auto pointer = address_space_r[page]; // Get the pointer to this page

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
	{
		return *(uint32_t *)(pointer + offset);
	}
    else if (address >= 0xBF000000 && address < (0xBFC00000))
    {
        // TODO: Minor hack for debugger not to exit
        return 0x00000000;
    }
    else if (address == 0x1F801010) {
        return 0x00000000;
    }
    else
    {
        std::string msg;
        msg = "32-bit read from unknown address: 0x" + format("{:08X}", address);
        Logger::error(msg.c_str());
        return Neo2::exit(1, Neo2::Subsystem::Bus);
    }
}

void Bus::fmem_write32(std::uint32_t address, std::uint32_t value)
{
    address = map_to_phys(address);
    const auto page = address >> 12;            // Divide the address by 4KB to get the page number.
    const auto offset = address & 0xFFF;        // The offset inside the 4KB page
    const auto pointer = address_space_w[page]; // Get the pointer to this page

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
    {
        *(uint32_t *)(pointer + offset) = value;
    }
    // Unknown register
    else if (address == 0x1000F500) {
        // NOP
    }
    else
    {
        std::string msg;
        msg = "32-bit write to unknown address: 0x" + format("{:08X}", address);
        Logger::error(msg.c_str());
        Neo2::exit(1, Neo2::Subsystem::Bus);
    }
}