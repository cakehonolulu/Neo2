#include <cstring>
#include <ee/ee.hh>
#include <ee/ee_jit.hh>
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

EE::EE(Bus* bus_, EmulationMode mode) : CPU(bus_, mode),
vu0(4 * 1024, 4 * 1024), vu1(16 * 1024, 16 * 1024),
jit(std::make_unique<EEJIT>(this))
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
    case EmulationMode::JIT:
        Logger::info("Running in JIT mode...");
        step_ = std::bind(&EEJIT::step, jit.get());
        break;
    default:
        Logger::error("Invalid emulation mode");
        exit(1);
        break;
    }

    reset();

    for (auto& opcode : opcodes) {
        opcode = [this](EE* cpu, std::uint32_t code) { this->unknown_opcode(code); };
    }
}

EE::~EE()
{
}

void EE::run(Breakpoint *breakpoints)
{
    run_(breakpoints);
}

void EE::step() {
    if (!Neo2::is_aborted()) {
        step_();
    }
}

void EE::reset() {
    pc = 0xBFC00000;
    next_pc = pc + 4;
    std::memset(registers, 0, sizeof(registers));
    std::memset(cop0.regs, 0, sizeof(cop0.regs));
    std::memset(fpr, 0, sizeof(fpr));
    lo.u128 = 0;
    hi.u128 = 0;

    cop0.PRId = 0x59;
    //cop0_registers[15] = 0x2E20;

    vu0.reset();
    vu1.reset();
};

std::uint32_t EE::fetch_opcode()
{
    return bus->read32(pc);
}

std::uint32_t EE::fetch_opcode(std::uint32_t pc_)
{
    return bus->read32(pc_);
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

void EE::set_backend(EmulationMode mode) {
    switch (mode)
    {
    case EmulationMode::Interpreter:
        Logger::info("Switching to Interpreter mode...");
        step_ = std::bind(&ee_step_interpreter, this);
        break;
    case EmulationMode::JIT:
        Logger::info("Switching to JIT mode...");
        step_ = std::bind(&EEJIT::step, jit.get());
        run_ = std::bind(&EEJIT::run, jit.get(), std::placeholders::_1);
        break;
    default:
        Logger::error("Invalid emulation mode");
        exit(1);
        break;
    }
}

void EE::load_elf(const std::string& elf_path) {
    std::ifstream elf_file(elf_path, std::ios::binary);

    if (!elf_file.is_open()) {
        Logger::error("Failed to open the ELF file: " + elf_path);
        return;
    }

    Logger::info("Loading ELF: " + elf_path);

    // Read ELF header
    Elf32_Ehdr header;
    elf_file.read(reinterpret_cast<char*>(&header), sizeof(header));

    if (std::memcmp(header.e_ident, "\x7F""ELF", 4) != 0) {
        Logger::error("Invalid ELF file: " + elf_path);
        return;
    }

    // Validate ELF format
    if (header.e_ident[EI_CLASS] != ELFCLASS32 || header.e_ident[EI_DATA] != ELFDATA2LSB) {
        Logger::error("Unsupported ELF format: must be 32-bit and little-endian.");
        return;
    }

    Logger::info("ELF header read successfully. Entry point: 0x" + format("{:08X}", header.e_entry));
    Logger::info("Program header offset: 0x" + format("{:08X}", header.e_phoff) +
                 ", number of program headers: " + format("{:d}", header.e_phnum));

    // Iterate through program headers
    elf_file.seekg(header.e_phoff, std::ios::beg);
    for (uint16_t i = 0; i < header.e_phnum; i++) {
        Elf32_Phdr phdr;
        elf_file.read(reinterpret_cast<char*>(&phdr), sizeof(phdr));

        if (phdr.p_type != PT_LOAD) {
            Logger::info("Skipping non-loadable segment. Type: " + format("{:08X}", phdr.p_type));
            continue;
        }

        Logger::info("Loading segment " + format("{:d}", i) + ":");
        Logger::info("  Virtual address: 0x" + format("{:08X}", phdr.p_vaddr));
        Logger::info("  File size: 0x" + format("{:X}", phdr.p_filesz));
        Logger::info("  Memory size: 0x" + format("{:X}", phdr.p_memsz));
        Logger::info("  Offset in file: 0x" + format("{:08X}", phdr.p_offset));

        // Translate virtual address to physical address
        uint32_t phys_addr = bus->map_to_phys(phdr.p_vaddr, bus->tlb);
        if (phys_addr + phdr.p_memsz > bus->ram.size()) {
            Logger::error("Segment exceeds RAM bounds. Skipping segment.");
            continue;
        }

        Logger::info("  Physical address: 0x" + format("{:08X}", phys_addr));

        // Copy segment data from ELF file into RAM
        elf_file.seekg(phdr.p_offset, std::ios::beg);
        elf_file.read(reinterpret_cast<char*>(bus->ram.data() + phys_addr), phdr.p_filesz);

        Logger::info("  Loaded " + format("{:X}", phdr.p_filesz) + " bytes to RAM at physical address 0x" +
                     format("{:08X}", phys_addr));

        // Zero-initialize the BSS (uninitialized data) if needed
        if (phdr.p_memsz > phdr.p_filesz) {
            std::memset(bus->ram.data() + phys_addr + phdr.p_filesz, 0, phdr.p_memsz - phdr.p_filesz);
            Logger::info("  Cleared BSS: 0x" + format("{:08X}", phys_addr + phdr.p_filesz) +
                         " to 0x" + format("{:08X}", phys_addr + phdr.p_memsz));
        }
    }

    // Set the ELF entry point
    sideload_elf = true;
    elf_entry_point = bus->map_to_phys(header.e_entry, bus->tlb);

    Logger::info("ELF entry point set to physical address 0x" + format("{:08X}", elf_entry_point));

    // Validate the entry point mapping
    if (elf_entry_point >= bus->ram.size()) {
        Logger::error("ELF entry point exceeds RAM bounds. Loading aborted.");
        sideload_elf = false;
        elf_entry_point = 0;
    }
}

void EE::set_elf_state(bool state) {
    jit->ee_jit_set_run_elf(state);
}
