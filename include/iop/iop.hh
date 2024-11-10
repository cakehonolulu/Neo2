#pragma once

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

    std::function<void()> step;
    void run() override;

    std::uint32_t fetch_opcode() override;
    void parse_opcode(std::uint32_t opcode) override;
    void unknown_opcode(std::uint32_t opcode);

    std::uint32_t registers[32];
    std::uint32_t pc;
    std::uint32_t next_pc;

    std::function<void(IOP*, std::uint32_t)> opcodes[256] = {&IOP::unknown_opcode};
};
