#pragma once

#include <bus/bus.hh>
#include <cstdint>
#include <ee/ee_interpreter.hh>
#include <functional>

enum class EmulationMode
{
    Interpreter,
    CachedInterpreter,
    DynamicRecompiler
};

union uint128_t {
    unsigned __int128 u128;
    std::uint64_t u64[2];
    std::uint32_t u32[4];
    std::uint16_t u16[8];
    std::uint8_t u8[16];
};

class EE
{
  private:
  public:
    EE(Bus *bus_, EmulationMode mode = EmulationMode::Interpreter);
    ~EE();

    std::function<void()> ee_step;

    void run();

    Bus *bus;

    std::uint32_t pc;
    std::uint32_t next_pc;

    uint128_t registers[32];
    std::uint32_t cop0_registers[32];
};
