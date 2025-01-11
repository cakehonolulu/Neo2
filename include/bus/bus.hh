#pragma once

#include <bus/tlb.hh>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

enum class BusMode
{
    Ranged,
    SoftwareFastMem
};

const uint32_t PAGE_SIZE = 4 * 1024; // Page size = 4KB

class Bus
{
  private:
  public:
    Bus(BusMode mode = BusMode::SoftwareFastMem);

    std::vector<std::uint8_t> bios;
    std::vector<std::uint8_t> ram;

    TLB tlb;
    uintptr_t *address_space_r;
    uintptr_t *address_space_w;

    std::function<std::uint32_t(std::uint32_t)> read32;
    std::function<void(std::uint32_t, std::uint32_t)> write32;
    void load_bios(const std::string &bios_path);
    void fmem_init();
    std::uint32_t fmem_read32(uint32_t address);
    void fmem_write32(uint32_t address, uint32_t value);
};
