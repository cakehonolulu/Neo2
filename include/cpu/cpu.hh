#pragma once

#include <cpu/breakpoint.hh>
#include <bus/bus.hh>
#include <cstdint>
#include <functional>
#include <log/log.hh>

enum class EmulationMode {
    Interpreter,
    CachedInterpreter,
    JIT
};

class CompiledBlock {
public:
    uint32_t start_pc;
    uint32_t end_pc;
    void* code_ptr;
    uint64_t last_used;
    bool contains_branch;
    std::string llvm_ir;
    uint32_t cycles;
};

class CPU {
public:
    CPU(Bus* bus_, EmulationMode mode) : bus(bus_), mode(mode) {}
    virtual ~CPU() = default;

    virtual void run(Breakpoint *breakpoints) = 0;
    virtual void step() = 0;
    virtual std::uint32_t fetch_opcode() = 0;
    virtual void parse_opcode(std::uint32_t opcode) = 0;

    virtual std::shared_ptr<const std::unordered_map<uint32_t, CompiledBlock>> get_block_cache() const = 0;

    Bus* bus;

    std::uint32_t pc;
    std::uint32_t next_pc;

  protected:
    EmulationMode mode;
};
