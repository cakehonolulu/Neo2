#include <iop/iop.hh>
#include <iop/iop_jit.hh>
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

IOP::IOP(Bus* bus_, EmulationMode mode) : CPU(bus_, mode), jit(std::make_unique<IOPJIT>(this)) {
    Logger::set_subsystem("IOP");

    switch (mode) {
        case EmulationMode::Interpreter:
            Logger::info("IOP running in Interpreter mode.");
            step_ = std::bind(&iop_step_interpreter, this);
            break;
        case EmulationMode::JIT:
            Logger::info("IOP running in JIT mode.");
            step_ = std::bind(&IOPJIT::step, jit.get());
            break;
        default:
            Logger::error("Unsupported IOP mode.");
            exit(1);
    }

    reset();

    for (auto& opcode : opcodes) {
        opcode = [this](IOP* /*cpu*/, std::uint32_t code) { this->unknown_opcode(code); };
    }
}

IOP::~IOP() {}

void IOP::run() {
    while (!Neo2::is_aborted()) {
        step();
    }
}

void IOP::step() {
    if (!Neo2::is_aborted()) {
        step_();
    }
}

void IOP::reset() {
    pc = 0xBFC00000;
    next_pc = pc + 4;

    std::memset(registers, 0, sizeof(registers));
    std::memset(cop0_registers, 0, sizeof(cop0_registers));

    cop0_registers[15] = 0x2;
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

void IOP::set_backend(EmulationMode mode) {
    switch (mode) {
        case EmulationMode::Interpreter:
            Logger::info("Switching IOP to Interpreter mode.");
            step_ = std::bind(&iop_step_interpreter, this);
            break;
        case EmulationMode::JIT:
            Logger::info("Switching IOP to JIT mode.");
            step_ = std::bind(&IOPJIT::step, jit.get());
            break;
        default:
            Logger::error("Unsupported IOP mode.");
            exit(1);
    }
}
