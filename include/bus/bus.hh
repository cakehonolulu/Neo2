#pragma once

#include <cstdint>
#include <string>
#include <vector>

class Bus
{

  public:
    Bus();

    std::vector<std::uint8_t> bios;
    std::vector<std::uint8_t> ram;

    uintptr_t *address_space_r;
    uintptr_t *address_space_w;

    void load_bios(const std::string &bios_path);
    std::uint32_t read32(uint32_t address);
};
