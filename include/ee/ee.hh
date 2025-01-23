#pragma once

#include <bus/bus.hh>
#include <cpu/cpu.hh>
#include <ee/vu/vu.hh>
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
    std::function<void(Breakpoint*)> run_;
    void run(Breakpoint *breakpoints) override;
    void step() override;
    void reset();

    std::shared_ptr<const std::unordered_map<uint32_t, CompiledBlock>> get_block_cache() const override {
      // Return a shared pointer to the block cache if JIT is available
      return jit ? jit->get_block_cache() : nullptr;
    }

    std::uint32_t fetch_opcode() override;
    std::uint32_t fetch_opcode(std::uint32_t pc_);
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

    std::uint64_t cycles = 0;

    VU vu0;
    VU vu1;

    std::uint32_t pc = 0xBFC00000;
    std::uint32_t next_pc;
    bool branching = false;
    bool likely_branch = false;
    std::uint32_t branch_dest;
    
  private:
    std::unique_ptr<EEJIT> jit;
};
