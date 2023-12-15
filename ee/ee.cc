#include <ee/ee.hh>
#include <neo2.hh>
#include <iostream>

#if __has_include(<format>)
    #include <format>
    using std::format;
#else
    #include <fmt/format.h>
    using fmt::format;
#endif

EE::EE(Bus *bus_)
{
    pc = 0x00000000;
    bus = bus_;
}

EE::~EE() {
}

void EE::run()
{
    while (true) {
        uint32_t opcode = fetchOpcode();

        parseOpcode(opcode);

        pc += 2;
    }
}

uint32_t EE::fetchOpcode()
{
    uint32_t opcode = (bus->read(pc + 1) << 8) | bus->read(pc);
    return opcode;
}

void EE::parseOpcode(uint32_t opcode)
{
    uint8_t function = (opcode >> 26) & 0x3F;

    switch (function)
    {
        default:
            std::cerr << BOLDRED << "[EE] Unimplemented opcode: 0x" << format("{:04X}", opcode) << " (Function bits: 0b" << format("{:04b}", function) << ")" << RESET << "\n";
            exit(1);
            break;
    }
}
