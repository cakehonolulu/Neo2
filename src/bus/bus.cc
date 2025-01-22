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

Bus::Bus(BusMode mode) : tlb(32), sio(), ee_intc(), gif(), gs(), timers(),
vif(), ipu()
{
	Logger::set_subsystem("BUS");

    // Scratchpad RAM (16 KB)
    scratchpad.resize(16 * 1024);

    // BIOS (4MB)
    bios.resize(1024 * 1024 * 4);

    // RAM (32MB)
    ram.resize(1024 * 1024 * 32);

    std::fill(bios.begin(), bios.end(), 0);

    std::fill(ram.begin(), ram.end(), 0);

    std::fill(scratchpad.begin(), scratchpad.end(), 0);

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

uint32_t Bus::map_to_phys(uint32_t vaddr, const TLB& tlb) {
    // Direct-mapped regions (kseg0, kseg1)
    if (vaddr >= 0x80000000 && vaddr < 0xA0000000) {
        // kseg0: Cached
        return vaddr & 0x1FFFFFFF;
    } else if (vaddr >= 0xA0000000 && vaddr < 0xC0000000) {
        // kseg1: Uncached
        return vaddr & 0x1FFFFFFF;
    }

    // TLB-mapped regions
    const TLBEntry* entry = tlb.find_entry(vaddr);
    
    if (entry) {
        // Check the ASID match if not global
        bool asid_match = (entry->global || (entry->asid == (vaddr >> 13 & 0xFF)));
        
        if (vaddr == 0xB0008000)
        {
            Logger::info("Checking TLB entry for vaddr 0x" + format("{:08X}", vaddr) + ". ASID match: " + format("{:X}", asid_match));
        }

        // If ASID matches, determine whether even or odd page is valid
        if (asid_match) {
            uint32_t vpn2_vaddr = (vaddr >> 13) & 0x7FFFF;  // Extract VPN2 from vaddr
            uint32_t page_offset = vaddr & 0xFFF;  // Extract page offset
                
            if (vaddr == 0xB0008000)
            {
                Logger::info("Virtual address 0x" + format("{:08X}", vaddr) + " corresponds to VPN2 0x" + format("{:X}", vpn2_vaddr));
            }

            // Check even page (pfn0)
            if ((vpn2_vaddr == (entry->vpn2 * 2)) && entry->v0) {
                uint32_t phys_addr = (entry->pfn0 << 12) | page_offset;

                if (vaddr == 0xB0008000)
                {
                    Logger::info("Even page valid, mapping to physical address 0x" + format("{:08X}", phys_addr));
                }

                return phys_addr;
            }
            // Check odd page (pfn1)
            if ((vpn2_vaddr == (entry->vpn2 * 2 + 1)) && entry->v1) {
                uint32_t phys_addr = (entry->pfn1 << 12) | page_offset;

                if (vaddr == 0xB0008000)
                {
                    Logger::info("Odd page valid, mapping to physical address 0x" + format("{:08X}", phys_addr));
                }

                return phys_addr;
            }

            // If no valid page found
            if (vaddr == 0xB0008000)
            {
                Logger::warn("No valid page found for virtual address 0x" + format("{:08X}", vaddr));
            }
        } else {
            if (vaddr == 0xB0008000)
            {
                Logger::warn("ASID mismatch for virtual address 0x" + format("{:08X}", vaddr));
            }
        }
    }

    // If no TLB entry matches, return the original virtual address
    //Logger::warn("Virtual address 0x" + format("{:08X}", vaddr) + " could not be translated to a physical address.");
    return vaddr;  // Default behavior; adjust as needed
}
