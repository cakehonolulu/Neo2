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
    void run() override;
    void step() override;
    void reset();

    std::uint32_t fetch_opcode() override;
    void parse_opcode(std::uint32_t opcode) override;
    void unknown_opcode(std::uint32_t opcode);
    void set_backend(EmulationMode mode);

    std::uint32_t registers[32];
    std::uint32_t cop0_registers[32];
    std::uint32_t pc = 0xBFC00000;
    std::uint32_t next_pc;
    bool branching = false;
    std::uint32_t branch_dest;

    std::function<void(IOP*, std::uint32_t)> opcodes[256];


  private:
    std::unique_ptr<IOPJIT> jit;
};
