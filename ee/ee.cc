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

EE::EE(Bus *bus_, EmulationMode mode)
{
    pc = 0xBFC00000;
    bus = bus_;

    switch (mode)
    {
    case EmulationMode::Interpreter:
        std::cout << CYAN << "[EE] Running in Interpreter mode..." << RESET "\n";
        ee_step = std::bind(&EE::ee_step_interpreter, this);
        break;
    case EmulationMode::CachedInterpreter:
        std::cerr << BOLDRED << "[EE] Cached interpreter mode is unavailable" << RESET "\n";
        exit(1);
        break;
    default:
        std::cerr << BOLDRED << "[EE] Invalid emulation mode" << RESET "\n";
        exit(1);
        break;
    }
}

EE::~EE()
{
}

void EE::run()
{
    while (true)
    {
        ee_step();
    }
}

void EE::ee_step_interpreter()
{
    uint32_t opcode = fetch_ee_opcode();

    parse_ee_opcode(opcode);

    pc += 2;
}

uint32_t inline EE::fetch_ee_opcode()
{
    return bus->read32(pc);
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
