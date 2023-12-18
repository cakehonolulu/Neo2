#include <bus/bus.hh>
#include <cassert>
#include <cstring>
#include <fstream>
#include <iostream>
#include <neo2.hh>

Bus::Bus()
{
    // BIOS (4MB)
    bios.resize(1024 * 1024 * 4);

    // RAM (32MB)
    ram.resize(1024 * 1024 * 32);

    // PS2's Address Space is 4GB, divide it in 4KB pages
    address_space_r = new uintptr_t[0x100000];
    address_space_w = new uintptr_t[0x100000];

    const uint32_t PAGE_SIZE = 4 * 1024; // Page size = 4KB

    std::fill(bios.begin(), bios.end(), 0);

    std::fill(ram.begin(), ram.end(), 0);

    memset(address_space_r, 0, sizeof(uintptr_t) * 0x100000);
    memset(address_space_w, 0, sizeof(uintptr_t) * 0x100000);

    // 4MB's of BIOS memory fit in 1024, 4KB pages
    for (auto pageIndex = 0; pageIndex < 1024; pageIndex++)
    {
        const auto pointer = (uintptr_t)&bios[(pageIndex * PAGE_SIZE)]; // pointer to page #pageIndex of the BIOS
        address_space_r[pageIndex + 0x1FC00] = pointer;                 // map this page to KUSEG BIOS
        address_space_r[pageIndex + 0x9FC00] = pointer;                 // Same for KSEG0
        address_space_r[pageIndex + 0xBFC00] = pointer;                 // Same for KSEG1
    }
}

void Bus::load_bios(const std::string &bios_path)
{
    std::ifstream bios_file(bios_path, std::ios::binary);

    if (!bios_file.is_open())
    {
        std::cerr << BOLDRED << "Failed to open the BIOS file: " << bios_path << RESET << "\n";
        return;
    }

    bios_file.read(reinterpret_cast<char *>(bios.data()), bios.size());

    assert(bios_file.good() && bios_file.gcount() == static_cast<std::streamsize>(bios.size()));

    bios_file.close();

    std::cout << BOLDBLUE << "BIOS file loaded successfully...!" << RESET << "\n";
}

std::uint32_t Bus::read32(std::uint32_t address)
{
    const auto page = address >> 12;            // Divide the address by 4KB to get the page number.
    const auto offset = address & 0xFFF;        // The offset inside the 4KB page
    const auto pointer = address_space_r[page]; // Get the pointer to this page

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
        return *(uint32_t *)(pointer +
                             offset); // Actually read the value using the pointer from the page table + the offset.

    else
    {
        // Handle other cases or throw an exception if needed
        printf("32-bit read from unknown address: 0x%08X", address);
        exit(1);
    }
}

std::uint8_t Bus::read(uint32_t address)
{
}
