#include <bus/bus.hh>
#include <log/log.hh>
#include <cassert>
#include <cstring>
#include <fstream>
#include <iostream>
#include <neo2.hh>

#if __has_include(<format>)
#include "neo2.hh"
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

Bus::Bus(BusMode mode) : tlb(32)
{
	Logger::set_subsystem("BUS");

    // BIOS (4MB)
    bios.resize(1024 * 1024 * 4);

    // RAM (32MB)
    ram.resize(1024 * 1024 * 32);

    std::fill(bios.begin(), bios.end(), 0);

    std::fill(ram.begin(), ram.end(), 0);

    switch (mode)
    {
    case BusMode::SoftwareFastMem:
        fmem_init();
        read8 = std::bind(&Bus::fmem_read8, this, std::placeholders::_1);
        read16 = std::bind(&Bus::fmem_read16, this, std::placeholders::_1);
        read32 = std::bind(&Bus::fmem_read32, this, std::placeholders::_1);
        read64 = std::bind(&Bus::fmem_read64, this, std::placeholders::_1);
        read128 = std::bind(&Bus::fmem_read128, this, std::placeholders::_1);
        write8 = std::bind(&Bus::fmem_write8, this, std::placeholders::_1, std::placeholders::_2);
        write16 = std::bind(&Bus::fmem_write16, this, std::placeholders::_1, std::placeholders::_2);
        write32 = std::bind(&Bus::fmem_write32, this, std::placeholders::_1, std::placeholders::_2);
        write64 = std::bind(&Bus::fmem_write64, this, std::placeholders::_1, std::placeholders::_2);
        write128 = std::bind(&Bus::fmem_write128, this, std::placeholders::_1, std::placeholders::_2);

        write32_dbg = std::bind(&Bus::fmem_write32_dbg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        Logger::info("Running Bus w/Software FastMem mode...");
        break;
    case BusMode::Ranged:
        Logger::error("Ranged Bus mode is unimplemented");
        exit(1);
        break;
    default:
        Logger::error("Invalid Bus mode");
        exit(1);
        break;
    }
}

void Bus::load_bios(const std::string &bios_path)
{
    std::ifstream bios_file(bios_path, std::ios::binary);

    if (!bios_file.is_open())
    {
        Logger::error("Failed to open the BIOS file: " + bios_path);
        return;
    }

    bios_file.read(reinterpret_cast<char *>(bios.data()), bios.size());

    assert(bios_file.good() && bios_file.gcount() == static_cast<std::streamsize>(bios.size()));

    bios_file.close();
}


std::uint32_t Bus::map_to_phys(std::uint32_t vaddr, const TLB& tlb) {
    // Check if the address is in KSEG0 or KSEG1
    if (vaddr >= 0x80000000 && vaddr < 0xA0000000) {
        //Logger::info("KSEG0: Directly mapped address, returning physical address.");
        return vaddr & 0x1FFFFFFF; // KSEG0: directly mapped, cached
    } else if (vaddr >= 0xA0000000 && vaddr < 0xC0000000) {
        //Logger::info("KSEG1: Directly mapped address, returning physical address.");
        return vaddr & 0x1FFFFFFF; // KSEG1: directly mapped, uncached
    } else {
        // Attempt to find the TLB entry for the virtual address
        const TLBEntry* entry = tlb.find_entry(vaddr);

        // If no entry is found, log the error
        if (!entry) {
            std::string msg = "TLB miss for address: 0x" + format("{:08X}", vaddr);
            Logger::error(msg.c_str());
            return 0;
        }

        // Calculate the virtual page number (VPN) and the offset
        uint32_t vpn2 = entry->entry_hi & ~(entry->page_mask);
        uint32_t offset = vaddr & 0xFFF; // Offset within the 4KB page

        // Determine if the address is mapped to entry_lo0 or entry_lo1
        uint32_t pfn;
        if (vaddr & (1 << 12)) {  // Use entry_lo1 for odd pages
            pfn = entry->entry_lo1 << 12;
        } else {  // Use entry_lo0 for even pages
            pfn = entry->entry_lo0 << 12;
        }

        uint32_t physical_address = pfn | offset;

        return physical_address;
    }
}
