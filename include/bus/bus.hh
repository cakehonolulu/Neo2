#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

enum class BusMode
{
    Ranged,
    SoftwareFastMem
};

class Bus
{
  private:
  public:
    Bus(BusMode mode = BusMode::SoftwareFastMem);

    std::vector<std::uint8_t> bios;
    std::vector<std::uint8_t> ram;

    uintptr_t *address_space_r;
    uintptr_t *address_space_w;

    std::function<std::uint32_t(std::uint32_t)> read32;
    void load_bios(const std::string &bios_path);
    void fmem_init();
    std::uint32_t fmem_read32(uint32_t address);
};
