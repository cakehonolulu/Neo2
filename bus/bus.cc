#include <bus/bus.hh>
#include <cassert>
#include <fstream>
#include <iostream>
#include <neo2.hh>

Bus::Bus()
{
    // BIOS (4MB)
    bios.resize(1024 * 1024 * 4);
}

void Bus::load_bios(const std::string &bios_path)
{
    std::ifstream bios_file(bios_path, std::ios::binary);

    if (!bios_file.is_open())
    {
        std::cerr << BOLDRED << "Failed to open the BIOS file: " << bios_path << RESET << "\n";
        return;
    }
    else
    {
        std::cout << BOLDBLUE << "BIOS file opened successfully...!" << RESET "\n";
    }

    bios_file.read(reinterpret_cast<char *>(bios.data()), bios.size());

    assert(bios_file.good() && bios_file.gcount() == static_cast<std::streamsize>(bios.size()));

    bios_file.close();
}

std::uint8_t Bus::read(uint32_t address)
{
}
