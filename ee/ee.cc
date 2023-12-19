#include <ee/ee.hh>
#include <iostream>
#include <neo2.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

EE::EE(Bus *bus_)
{
    pc = 0xBFC00000;
    bus = bus_;
}

EE::~EE()
{
}

void EE::run()
{
    while (true)
    {
        uint32_t opcode = fetch_ee_opcode();

        parse_ee_opcode(opcode);

        pc += 2;
    }
}

uint32_t EE::fetch_ee_opcode()
{
    uint32_t opcode = bus->read32(pc);
    return opcode;
}

void EE::parse_ee_opcode(uint32_t opcode)
{
    uint8_t function = (opcode >> 26) & 0x3F;

    switch (function)
    {
    default:
        std::cerr << BOLDRED << "[EE] Unimplemented opcode: 0x" << format("{:04X}", opcode) << " (Function bits: 0x"
                  << format("{:02X}", function) << ")" << RESET << "\n";
        exit(1);
        break;
    }
}
