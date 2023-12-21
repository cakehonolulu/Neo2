#include <ee/ee_interpreter.hh>
#include <neo2.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

void ee_step_interpreter(EE *ee)
{
    std::uint32_t opcode = fetch_ee_opcode(ee);
    ee->pc = ee->next_pc;
    ee->next_pc += 4;
    parse_ee_opcode(ee, opcode);
}

std::uint32_t inline fetch_ee_opcode(EE *ee)
{
    return ee->bus->read32(ee->pc);
}

void parse_ee_opcode(EE *ee, std::uint32_t opcode)
{
    std::uint8_t function = (opcode >> 26) & 0x3F;

    switch (function)
    {
    default:
        std::cerr << BOLDRED << "[EE] Unimplemented opcode: 0x" << format("{:04X}", opcode) << " (Function bits: 0x"
                  << format("{:02X}", function) << ")" << RESET << "\n";
        exit(1);
        break;
    }
}
