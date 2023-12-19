#pragma once

#include <bus/bus.hh>
#include <cstdint>

class EE
{

  public:
    EE(Bus *bus_);
    ~EE();

    void run();

    Bus *bus;
    std::uint32_t pc;

    std::uint32_t fetch_ee_opcode();
    void parse_ee_opcode(std::uint32_t opcode);
};
