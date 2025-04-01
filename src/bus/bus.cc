#include "ee/intc/intc.hh"
#include <bus/bus.hh>
#include <log/log.hh>
#include <cassert>
#include <cstring>
#include <fstream>
#include <iostream>
#include <neo2.hh>
#include <openssl/evp.h>
#include <iomanip>
#include <sstream>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

static std::string compute_md5(const std::vector<uint8_t>& data) {
    const EVP_MD* md5 = EVP_md5();
    unsigned char md5_result[EVP_MAX_MD_SIZE];
    unsigned int md5_length = 0;

    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (!ctx) {
        Logger::error("Failed to create EVP_MD_CTX for MD5 computation");
        return "";
    }

    if (EVP_DigestInit_ex(ctx, md5, nullptr) != 1 ||
        EVP_DigestUpdate(ctx, data.data(), data.size()) != 1 ||
        EVP_DigestFinal_ex(ctx, md5_result, &md5_length) != 1) {
        Logger::error("Failed to compute MD5 checksum using EVP API");
        EVP_MD_CTX_free(ctx);
        return "";
    }

    EVP_MD_CTX_free(ctx);

    std::ostringstream md5_string;
    for (unsigned int i = 0; i < md5_length; ++i) {
        md5_string << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(md5_result[i]);
    }
    return md5_string.str();
}

static const std::unordered_map<std::string, std::function<void(Bus&)>> bios_quirks = {
    // DESR 5000 1.80 BIOS
    {"1c6cd089e6c83da618fbf2a081eb4888", [](Bus& bus) {
        bus.dmac.D_ENABLER = 0x1201;
        bus.ram.resize(1024 * 1024 * 64); // Resize to 64MB
        std::fill(bus.ram.begin(), bus.ram.end(), 0);

    }},
    // SCPH-10000 BIOS
    {"acf4730ceb38ac9d8c7d8e21f2614600", [](Bus& bus) {
    }},
};

Bus::Bus(BusMode mode)
    : tlb(48), sio(), ee_intc(), gif(*this), gs(), timers(), iop_timers(), vif(), ipu(), dmac(*this), rdram(),
      iop_dmac(), sio2(), sif(), cdvd(), firewire()
{
    Logger::set_subsystem("BUS");

    // Scratchpad RAM (16 KB)
    scratchpad.resize(16 * 1024);

    // BIOS (4MB)
    bios.resize(1024 * 1024 * 4);

    // RAM (32MB) - Default size
    ram.resize(1024 * 1024 * 32);
    std::fill(ram.begin(), ram.end(), 0);

    // IOP RAM (2MB)
    iop_ram.resize(1024 * 1024 * 2);

    std::fill(bios.begin(), bios.end(), 0);
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

    // Compute MD5 checksum and apply quirks if needed
    std::string md5_checksum = compute_md5(bios);
    Logger::info("BIOS MD5 checksum: " + md5_checksum);

    auto it = bios_quirks.find(md5_checksum);
    if (it != bios_quirks.end()) {
        Logger::info("Applying quirks for BIOS MD5 checksum: " + md5_checksum);
        it->second(*this);
    } else {
        Logger::warn("Unknown BIOS MD5 checksum: " + md5_checksum);
    }

    // PS2's Address Space is 4GB, divided into 4KB pages
    address_space_r = new uintptr_t[0x100000]; // Readable memory pages (4KB * 1024 * 1024 = 4GB)
    address_space_w = new uintptr_t[0x100000]; // Writable memory pages

    memset(address_space_r, 0, sizeof(uintptr_t) * 0x100000);
    memset(address_space_w, 0, sizeof(uintptr_t) * 0x100000);

    // Map BIOS pages into KUSEG, KSEG0, and KSEG1 (uncached and cached)
    for (auto pageIndex = 0; pageIndex < 1024; pageIndex++)
    {
        const auto pointer = (uintptr_t)&bios[(pageIndex * PAGE_SIZE)]; // pointer to page #pageIndex of the BIOS
        // Map BIOS to KUSEG, KSEG0, KSEG1
        address_space_r[pageIndex + 0x1FC00] = pointer; // KUSEG BIOS
        address_space_r[pageIndex + 0x9FC00] = pointer; // KSEG0 BIOS (cached)
        address_space_r[pageIndex + 0xBFC00] = pointer; // KSEG1 BIOS (uncached)
    }

    // Main RAM: 32 MB starting at physical address 0x00000000
    for (auto pageIndex = 0; pageIndex < 0x8000; pageIndex++)
    {                                                                  // 32 MB / 4KB per page = 0x8000 pages
        const auto pointer = (uintptr_t)&ram[(pageIndex * PAGE_SIZE)]; // Main RAM pointer
        address_space_r[pageIndex] = pointer;                          // Map to KUSEG main RAM
        address_space_w[pageIndex] = pointer;                          // Map to KSEG0/KSEG1 main RAM (can write to it)
    }

    // IOP RAM: 2 MB starting at physical address 0x1C000000
    for (auto pageIndex = 0; pageIndex < 0x200; pageIndex++)
    {                                                                      // 2 MB / 4KB per page = 0x200 pages
        const auto pointer = (uintptr_t)&iop_ram[(pageIndex * PAGE_SIZE)]; // IOP RAM pointer
        address_space_r[pageIndex + 0x1C000] = pointer;                    // Map to IOP RAM
        address_space_w[pageIndex + 0x1C000] = pointer;                    // Map to IOP RAM (can write to it)
    }
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
