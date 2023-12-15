#pragma once

#include <cstdint>
#include <string>
#include <vector>

class Bus
{

  public:
    Bus();

    std::vector<std::uint8_t> bios;

    void load_bios(const std::string &bios_path);
    std::uint8_t read(uint32_t address);
};
