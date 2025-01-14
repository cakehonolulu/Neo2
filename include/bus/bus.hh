#pragma once

#include <bus/tlb.hh>
#include <reg.hh>
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

    std::uint32_t map_to_phys(std::uint32_t vaddr, const TLB &tlb);

    std::function<std::uint8_t(std::uint32_t)> read8;
    std::function<std::uint32_t(std::uint32_t)> read32;
    std::function<uint128_t(std::uint32_t)> read128;
    std::function<void(std::uint32_t, std::uint8_t)> write8;
    std::function<void(std::uint32_t, std::uint32_t)> write32;
    std::function<void(std::uint32_t, std::uint64_t)> write64;
    std::function<void(std::uint32_t, uint128_t)> write128;
    void load_bios(const std::string &bios_path);
    void fmem_init();
    std::uint8_t fmem_read8(uint32_t address);
    std::uint32_t fmem_read32(uint32_t address);
    uint128_t fmem_read128(uint32_t address);
    void fmem_write8(uint32_t address, uint8_t value);
    void fmem_write32(uint32_t address, uint32_t value);
    void fmem_write64(uint32_t address, uint64_t value);
    void fmem_write128(uint32_t address, uint128_t value);
};
