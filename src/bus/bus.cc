#include "ee/intc/intc.hh"
#include <bus/bus.hh>
#include <log/log.hh>
#include <cassert>
#include <cstring>
#include <fstream>
#include <iostream>
#include <neo2.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

Bus::Bus(BusMode mode)
    : tlb(32), sio(), ee_intc(), gif(*this), gs(), timers(), iop_timers(), vif(), ipu(), dmac(*this), rdram() // Pass reference to Bus instance
{
    Logger::set_subsystem("BUS");

    // Scratchpad RAM (16 KB)
    scratchpad.resize(16 * 1024);

    // BIOS (4MB)
    bios.resize(1024 * 1024 * 4);

    // RAM (32MB)
    ram.resize(1024 * 1024 * 32);

    // IOP RAM (2MB
    iop_ram.resize(1024 * 1024 * 2);

    std::fill(bios.begin(), bios.end(), 0);

    std::fill(ram.begin(), ram.end(), 0);

    std::fill(scratchpad.begin(), scratchpad.end(), 0);

    std::fill(iop_ram.begin(), iop_ram.end(), 0);

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

uint32_t Bus::map_to_phys(uint32_t vaddr, const TLB &tlb)
{
    // Direct-mapped regions (kseg0, kseg1)
    if (vaddr >= 0x80000000 && vaddr < 0xC0000000)
    {
        return vaddr & 0x1FFFFFFF;
    }

    // TLB-mapped regions
    const TLBEntry *entry = tlb.find_entry(vaddr);
    if (entry)
    {
        // Determine effective page size.
        // The PageMask field (lower 12 bits) is stored directly in the TLBEntry.
        // For a 4KB page, PageMask is 0; then page_size = (0+1) << 12 = 4096.
        // For a 16KB page, PageMask might be 0x3; then page_size = (3+1) << 12 = 16384.
        uint32_t effective_mask = entry->page_mask & 0xFFF;
        uint32_t page_size = (effective_mask + 1) << 12;

        // Compute the number of offset bits = log2(page_size).
        uint32_t shift = 0;
        for (uint32_t temp = page_size; temp > 1; temp >>= 1)
        {
            shift++;
        }

        // Compute page offset from vaddr.
        uint32_t page_offset = vaddr & ((1u << shift) - 1);
        // Compute full virtual page number.
        uint32_t vpn = vaddr >> shift;
        // The least-significant bit of vpn indicates even (0) or odd (1) page.
        if ((vpn & 1u) == 0)
        {
            if (entry->v0)
            {
                uint32_t phys_addr = (entry->pfn0 << shift) | page_offset;
                return phys_addr;
            }
        }
        else
        {
            if (entry->v1)
            {
                uint32_t phys_addr = (entry->pfn1 << shift) | page_offset;
                return phys_addr;
            }
        }
    }
    // If no matching TLB entry is found (or invalid), return vaddr.
    return vaddr;
}
