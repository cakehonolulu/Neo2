#pragma once

#include <reg.hh>
#include <cstdint>

class Bus;

void fmem_init(Bus &bus);
std::uint32_t fmem_read32(Bus &bus, std::uint32_t address);
uint128_t fmem_read128(Bus &bus, uint32_t address);

std::uint32_t fmem_write32(Bus &bus, std::uint32_t address, std::uint32_t value);
std::uint32_t fmem_write64(Bus &bus, std::uint32_t address, uint128_t value);
std::uint32_t fmem_write128(Bus &bus, std::uint32_t address, uint128_t value);
