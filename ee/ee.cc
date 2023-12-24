#include <cstring>
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
    bus = bus_;

    switch (mode)
    {
    case EmulationMode::Interpreter:
        std::cout << CYAN << "[EE] Running in Interpreter mode..." << RESET "\n";
        ee_interpreter_setup(this);
        ee_step = std::bind(&ee_step_interpreter, this);
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

    std::memset(registers, 0, sizeof(registers));
    pc = 0xBFC00000;
    next_pc = pc + 4;
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

std::uint32_t EE::ee_fetch_opcode()
{
    return bus->read32(pc);
}

void EE::ee_parse_opcode(std::uint32_t opcode)
{
    std::uint8_t function = (opcode >> 26) & 0x3F;

    opcodes[function](this, opcode);
}

void EE::ee_unknown_opcode(std::uint32_t opcode)
{
    std::cerr << BOLDRED << "[EE] Unimplemented opcode: 0x" << format("{:04X}", opcode) << " (Function bits: 0x"
              << format("{:02X}", (opcode >> 26) & 0x3F) << ")" << RESET << "\n";
    exit(1);
}
