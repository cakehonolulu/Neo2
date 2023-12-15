#pragma once

#include <string>
#include <vector>
#include <cstdint>

class Bus {

public:

    Bus();

    std::vector<std::uint8_t> memory_map;

    void load_bios(const std::string& bios_path);
    std::uint8_t read(uint32_t address);
    
};
