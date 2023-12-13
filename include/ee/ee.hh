#pragma once

#include <memory/memory.hh>
#include <cstdint>

class EE {

public:
    EE(Memory *memory_);
    ~EE();

    void run();

    Memory *memory;
    std::uint32_t pc;

    std::uint32_t fetchOpcode();
    void parseOpcode(std::uint32_t opcode);
};
