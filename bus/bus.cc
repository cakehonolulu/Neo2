#include <bus/bus.hh>
#include <fstream>
#include <iostream>
#include <neo2.hh>

Bus::Bus()
{
    // BIOS (4MB)
    memory_map.resize(0x00400000, 0); // 0x00200000 = 2MB
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

    const uint32_t bios_base_addr = 0x00000000;
    const uint32_t bios_end_addr = bios_base_addr + 0x03FFFFFF;
    uint32_t offset = bios_base_addr;

    while (!bios_file.eof() && offset <= bios_end_addr)
    {
        char byte;
        bios_file.read(&byte, 1);
        memory_map[offset] = static_cast<uint8_t>(byte);
        offset++;
    }

    bios_file.close();
}

std::uint8_t Bus::read(uint32_t address)
{
    if (address < memory_map.size())
    {
        return memory_map[address];
    }
    else
    {
        return 0;
    }
}
