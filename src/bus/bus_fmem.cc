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

void Bus::fmem_init() {
    tlb = TLB(32);

    // Initialize TLB entries for BIOS and RAM
    TLBEntry entry;

    // KSEG1 (uncached BIOS)
    entry.page_mask = 0x1FFF; // 4KB pages
    entry.entry_hi = 0xBFC00000; // Virtual address
    entry.entry_lo0 = 0x1FC00000 >> 12; // Physical address shifted (PFN for 0x1FC00000)
    entry.entry_lo1 = 0x1FC01000 >> 12; // Next page
    entry.global = true;
    tlb.write_entry(0, entry);

    // KSEG0 (cached BIOS)
    entry.entry_hi = 0x9FC00000;
    entry.entry_lo0 = 0x1FC00000 >> 12; // Physical address shifted (PFN for 0x1FC00000)
    entry.entry_lo1 = 0x1FC01000 >> 12; // Next page
    tlb.write_entry(1, entry);

    // KUSEG (RAM)
    entry.page_mask = 0x007FE000; // 4MB pages
    entry.entry_hi = 0x00000000;
    entry.entry_lo0 = 0x00000000 >> 12; // Physical address shifted (PFN for 0x00000000)
    entry.entry_lo1 = 0x00400000 >> 12; // For next 4MB page
    entry.global = true;
    tlb.write_entry(2, entry);

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

std::uint32_t map_to_phys(std::uint32_t vaddr, const TLB& tlb, const uintptr_t* address_space) {
    if (vaddr >= 0x80000000 && vaddr < 0xA0000000) {
        return vaddr & 0x1FFFFFFF; // KSEG0: directly mapped, cached
    } else if (vaddr >= 0xA0000000 && vaddr < 0xC0000000) {
        return vaddr & 0x1FFFFFFF; // KSEG1: directly mapped, uncached
    } else {
        const TLBEntry* entry = tlb.find_entry(vaddr);
        if (entry) {
            uint32_t vpn2 = entry->entry_hi & ~(entry->page_mask);
            uint32_t offset = vaddr & 0xFFF; // Offset within the 4KB page

            uint32_t pfn;
            if (vaddr & (1 << 12)) { // Use entry_lo1 for odd pages
                pfn = entry->entry_lo1 << 12;
            } else { // Use entry_lo0 for even pages
            pfn = entry->entry_lo0 << 12;
        }

        return pfn | (vaddr & 0xFFF);
        
        } else {
            std::string msg = "Unhandled address translation for: 0x" + format("{:08X}", vaddr);
            Logger::error(msg.c_str());
            return 0;
        }
    }
}

std::uint32_t Bus::fmem_read32(std::uint32_t address)
{
    address = map_to_phys(address, tlb, address_space_r);
    const auto page = address >> 12;            // Divide the address by 4KB to get the page number.
    const auto offset = address & 0xFFF;        // The offset inside the 4KB page
    const auto pointer = address_space_r[page]; // Get the pointer to this page

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
    {
        return *(uint32_t *)(pointer + offset);
    }
    else if (address >= 0xBF000000 && address < 0xBFC00000)
    {
        // TODO: Minor hack for debugger not to exit
        return 0x00000000;
    }
    else if (address == 0x1F801010) {
        return 0x00000000;
    }
    else
    {
        std::string msg = "32-bit read from unknown address: 0x" + format("{:08X}", address);
        Logger::error(msg.c_str());
        return Neo2::exit(1, Neo2::Subsystem::Bus);
    }
}

void Bus::fmem_write32(std::uint32_t address, std::uint32_t value)
{
    address = map_to_phys(address, tlb, address_space_w);
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
    else if (address == 0x1F801010) {
        // NOP
    }
    else
    {
        std::string msg = "32-bit write to unknown address: 0x" + format("{:08X}", address);
        Logger::error(msg.c_str());
        Neo2::exit(1, Neo2::Subsystem::Bus);
    }
}
