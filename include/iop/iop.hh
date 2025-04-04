#pragma once

#include "iop/iop_jit.hh"
#include <bus/bus.hh>
#include <cstdint>
#include <cpu/cpu.hh>
#include <iop/iop_interpreter.hh>
#include <functional>

class IOP : public CPU
{
public:
    IOP(Bus *bus_, EmulationMode mode = EmulationMode::Interpreter);
    ~IOP();

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

    std::uint32_t registers[32] __attribute__((__packed__));
    std::uint32_t cop0_registers[32] __attribute__((__packed__));
    std::uint32_t lo = 0;
    std::uint32_t hi = 0;
    std::uint32_t pc = 0xBFC00000;
    std::uint32_t next_pc;
    bool branching = false;
    std::uint32_t branch_dest;

    std::uint64_t cycles = 0;

    std::function<void(IOP*, std::uint32_t)> opcodes[256];

    void execute_cycles(uint64_t cycle_limit, Breakpoint *breakpoints);

  private:
    std::unique_ptr<IOPJIT> jit;
};
