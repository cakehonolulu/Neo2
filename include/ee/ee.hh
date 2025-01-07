#pragma once

#include <bus/bus.hh>
#include <cpu/cpu.hh>
#include <cstdint>
#include <ee/ee_interpreter.hh>
#include <functional>
#include <memory>
#include <ee/ee_jit.hh>

union uint128_t {
    unsigned __int128 u128;
    std::uint64_t u64[2];
    std::uint32_t u32[4];
    std::uint16_t u16[8];
    std::uint8_t u8[16];
};

class EE : public CPU
{
  private:
    std::unique_ptr<EEJIT> jit;

  public:
    EE(Bus *bus_, EmulationMode mode = EmulationMode::Interpreter);
    ~EE();

    std::function<void()> step_;
    void run() override;
    void step() override;
    void reset();

    std::uint32_t fetch_opcode() override;
    void parse_opcode(std::uint32_t opcode) override;
    void unknown_opcode(std::uint32_t opcode);
    void set_backend(EmulationMode mode);

    // 104 MIPS III/IV Instructions
    // 111 EE-Specific (SIMD-Like) Instructions
    std::function<void(EE *, std::uint32_t)> opcodes[104 + 111];

    uint128_t registers[32];
    std::uint32_t cop0_registers[32];

    std::uint32_t pc = 0xBFC00000;
    std::uint32_t next_pc;
};
