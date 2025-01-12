#pragma once

#include <cstdint>

union uint128_t {
    unsigned __int128 u128;
    std::uint64_t u64[2];
    std::uint32_t u32[4];
    std::uint16_t u16[8];
    std::uint8_t u8[16];
};
