#include <iop/iop.hh>
#include <log/log.hh>
#include <cstring>

#if __has_include(<format>)
#include "neo2.hh"
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

IOP::IOP(Bus* bus_, EmulationMode mode) : CPU(bus_, mode) {
    Logger::set_subsystem("IOP");

    switch (mode) {
        case EmulationMode::Interpreter:
            Logger::info("IOP running in Interpreter mode.");
            step_ = std::bind(&iop_step_interpreter, this);
            break;
        default:
            Logger::error("Unsupported IOP mode.");
            exit(1);
    }

    std::memset(registers, 0, sizeof(registers));
    pc = 0xBFC00000;
    next_pc = pc + 4;

    for (auto& opcode : opcodes) {
        opcode = [this](IOP* cpu, std::uint32_t code) { this->unknown_opcode(code); };
    }
}

IOP::~IOP() {}

void IOP::run() {
    while (!Neo2::is_aborted()) {
        step();
    }
}

void IOP::step() {
    step_();
}

void IOP::reset() {
    pc = 0xBFC00000;
    old_pc = 0xBFC00000;
    next_pc = pc + 4;
    std::memset(registers, 0, sizeof(registers));
};

std::uint32_t IOP::fetch_opcode() {
    return bus->read32(pc);
}

void IOP::parse_opcode(std::uint32_t opcode)
{
    std::uint8_t function = (opcode >> 26) & 0x3F;

    opcodes[function](this, opcode);
}

void IOP::unknown_opcode(std::uint32_t opcode) {
    Logger::error("Unimplemented IOP opcode: 0x" + format("{:04X}", opcode) + " (Function bits: 0x"
              + format("{:02X}", (opcode >> 26) & 0x3F) + ")");
    Neo2::exit(1, Neo2::Subsystem::IOP);
}
