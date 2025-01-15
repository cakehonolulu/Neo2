#pragma once

#include <bus/bus.hh>
#include <cpu/cpu.hh>
#include <cstdint>
#include <ee/ee_interpreter.hh>
#include <functional>
#include <memory>
#include <ee/ee_jit.hh>
#include <reg.hh>

class EE : public CPU
{
  public:
    EE(Bus *bus_, EmulationMode mode = EmulationMode::Interpreter);
    ~EE();

    std::function<void()> step_;
    void run() override;
    void step() override;
    void reset();

    const std::unordered_map<uint32_t, CompiledBlock>* get_block_cache() const override {
      return jit ? &jit->block_cache : nullptr;
    }

    std::uint32_t fetch_opcode() override;
    void parse_opcode(std::uint32_t opcode) override;
    void unknown_opcode(std::uint32_t opcode);
    void set_backend(EmulationMode mode);

    // 104 MIPS III/IV Instructions
    // 111 EE-Specific (SIMD-Like) Instructions
    std::function<void(EE *, std::uint32_t)> opcodes[104 + 111];

    uint128_t registers[32];
    std::uint32_t cop0_registers[32];
    fpu_reg_t fpr[32];

    uint128_t lo;
    uint128_t hi;

    std::uint32_t pc = 0xBFC00000;
    std::uint32_t next_pc;
    bool branching = false;
    std::uint32_t branch_dest;
    
  private:
    std::unique_ptr<EEJIT> jit;
};
