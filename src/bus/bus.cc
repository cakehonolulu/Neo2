#include <bus/bus.hh>
#include <log/log.hh>
#include <cassert>
#include <cstring>
#include <fstream>
#include <iostream>
#include <neo2.hh>

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
        read32 = std::bind(&Bus::fmem_read32, this, std::placeholders::_1);
        write32 = std::bind(&Bus::fmem_write32, this, std::placeholders::_1, std::placeholders::_2);
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
