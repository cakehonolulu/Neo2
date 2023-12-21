#include <bus/bus.hh>
#include <cassert>
#include <cstring>
#include <fstream>
#include <iostream>
#include <neo2.hh>

Bus::Bus(BusMode mode)
{
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
        read32 = std::bind(&Bus::fmem_read32, this, std::placeholders::_1);
        std::cout << CYAN << "[BUS] Running Bus w/Software FastMem mode..." << RESET "\n";
        break;
    case BusMode::Ranged:
        std::cerr << BOLDRED << "[BUS] Ranged Bus mode is unimplemented" << RESET "\n";
        exit(1);
        break;
    default:
        std::cerr << BOLDRED << "[BUS] Invalid Bus mode" << RESET "\n";
        exit(1);
        break;
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
}
