#include <cstring>
#include <ee/ee.hh>
#include <log/log.hh>
#include <iostream>
#include <neo2.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

EE::EE(Bus* bus_, EmulationMode mode) : CPU(bus_, mode)
{
	Logger::set_subsystem("EE");

    switch (mode)
    {
    case EmulationMode::Interpreter:
        Logger::info("Running in Interpreter mode...");
        ee_interpreter_setup(this);
        step_ = std::bind(&ee_step_interpreter, this);
        break;
    case EmulationMode::CachedInterpreter:
        Logger::error("Cached interpreter mode is unavailable");
        exit(1);
        break;
    default:
        Logger::error("Invalid emulation mode");
        exit(1);
        break;
    }

    std::memset(registers, 0, sizeof(registers));
    pc = 0xBFC00000;
    next_pc = pc + 4;

    for (auto& opcode : opcodes) {
        opcode = [this](EE* cpu, std::uint32_t code) { this->unknown_opcode(code); };
    }
}

EE::~EE()
{
}

void EE::run()
{
    while (!Neo2::is_aborted())
    {
        step();
    }
}

void EE::step() {
    step_();
}

void EE::reset() {
    pc = 0xBFC00000;
    old_pc = 0xBFC00000;
    next_pc = pc + 4;
    std::memset(registers, 0, sizeof(registers));
};

std::uint32_t EE::fetch_opcode()
{
    return bus->read32(pc);
}

void EE::parse_opcode(std::uint32_t opcode)
{
    std::uint8_t function = (opcode >> 26) & 0x3F;

    opcodes[function](this, opcode);
}

void EE::unknown_opcode(std::uint32_t opcode)
{
    Logger::error("Unimplemented EE opcode: 0x" + format("{:04X}", opcode) + " (Function bits: 0x"
              + format("{:02X}", (opcode >> 26) & 0x3F) + ")");
    Neo2::exit(1, Neo2::Subsystem::EE);
}
