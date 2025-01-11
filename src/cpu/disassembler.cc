#include <cpu/disassembler.hh>
#include <sstream>
#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

// Register names
const std::string mips_register_names[32] = {
    "zr", "at", "v0", "v1", "a0", "a1", "a2", "a3",
    "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
    "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
    "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"
};

const std::string cop0_register_names[32] = {
    "Index", "Random", "EntryLo0", "EntryLo1", "Context", "PageMask", "Wired", "",
    "BadVAddr", "Count", "EntryHi", "Compare", "Status", "Cause", "EPC", "PRId",
    "Config", "LLAddr", "WatchLo", "WatchHi", "XContext", "", "", "", "",
    "", "", "", "", "", "", ""
};

// Constructor
Disassembler::Disassembler()
{
    initialize_opcode_table();
    initialize_pseudo_opcode_table();
}

// Initialize opcode tables
void Disassembler::initialize_opcode_table() {
    // Base opcode table (main instructions using the primary opcode)
    opcode_mnemonics[0x02] = OpcodeEntry("j", InstructionType::JType, true);
    opcode_mnemonics[0x03] = OpcodeEntry("jal", InstructionType::JType, true);
    opcode_mnemonics[0x04] = OpcodeEntry("beq", InstructionType::IType, true);
    opcode_mnemonics[0x05] = OpcodeEntry("bne", InstructionType::IType, true);
    opcode_mnemonics[0x08] = OpcodeEntry("addi", InstructionType::IType);
    opcode_mnemonics[0x09] = OpcodeEntry("addiu", InstructionType::IType);
    opcode_mnemonics[0x0A] = OpcodeEntry("slti", InstructionType::IType);
    opcode_mnemonics[0x0B] = OpcodeEntry("sltiu", InstructionType::IType);
    opcode_mnemonics[0x0C] = OpcodeEntry("andi", InstructionType::RType);
    opcode_mnemonics[0x0D] = OpcodeEntry("ori", InstructionType::IType);
    opcode_mnemonics[0x0F] = OpcodeEntry("lui", InstructionType::IType);
    opcode_mnemonics[0x23] = OpcodeEntry("lw", InstructionType::IType);
    opcode_mnemonics[0x2B] = OpcodeEntry("sw", InstructionType::IType);

    // Extended opcode table (e.g., special opcodes when function == 0x00)
    extended_opcodes[0x00] = OpcodeEntry("sll", InstructionType::RType);
    extended_opcodes[0x08] = OpcodeEntry("jr", InstructionType::JType, true);
    extended_opcodes[0x0F] = OpcodeEntry("sync", InstructionType::Unknown);
    extended_opcodes[0x20] = OpcodeEntry("add", InstructionType::RType);
    extended_opcodes[0x21] = OpcodeEntry("addu", InstructionType::RType);
    extended_opcodes[0x22] = OpcodeEntry("sub", InstructionType::RType);
    extended_opcodes[0x23] = OpcodeEntry("subu", InstructionType::RType);
    extended_opcodes[0x24] = OpcodeEntry("and", InstructionType::RType);
    extended_opcodes[0x25] = OpcodeEntry("or", InstructionType::RType);
    extended_opcodes[0x26] = OpcodeEntry("xor", InstructionType::RType);
    extended_opcodes[0x27] = OpcodeEntry("nor", InstructionType::RType);
    extended_opcodes[0x2A] = OpcodeEntry("slt", InstructionType::RType);
    extended_opcodes[0x2B] = OpcodeEntry("sltu", InstructionType::RType);
    extended_opcodes[0x43] = OpcodeEntry("srl", InstructionType::RType);

    // COP0 opcode table
    cop_opcode_tables[0x10][0x00] = OpcodeEntry("mfc0", InstructionType::IType);
    cop_opcode_tables[0x10][0x04] = OpcodeEntry("mtc0", InstructionType::IType);
    cop_opcode_tables[0x10][0x10] = OpcodeEntry("tlbwi", InstructionType::IType);
}

// Initialize pseudoinstructions
void Disassembler::initialize_pseudo_opcode_table() {
    // Example pseudoinstructions
    // Pseudoinstruction: nop -> sll $zr, $zr, 0
    pseudo_opcodes.push_back(PseudoOpcode{
        0xFC00003F, // Mask: check all bits except shift amount and rs/rt fields
        0x00000000, // Expected: all zeroes for nop
        "nop",
        [](DisassemblyData& data, uint32_t opcode) {
            data.mnemonic = "nop";
            data.operands.clear(); // nop has no operands
        }
    });

    // Pseudoinstruction: move $rd, $rs -> addu $rd, $rs, $zr
    pseudo_opcodes.push_back(PseudoOpcode{
        0xFC0007FF, // Mask: opcode and function bits
        0x00000020, // Expected: 'addu' function code
        "move",
        [](DisassemblyData& data, uint32_t opcode) {
            uint8_t rd = (opcode >> 11) & 0x1F;
            uint8_t rs = (opcode >> 21) & 0x1F;

            data.mnemonic = "move";
            data.operands.clear();
            data.operands.push_back({"$" + mips_register_names[rd], rd});
            data.operands.push_back({"$" + mips_register_names[rs], rs});
        }
    });

    // Pseudoinstruction: clear $rd -> addu $rd, $zr, $zr
    pseudo_opcodes.push_back(PseudoOpcode{
        0xFFC007FF, // Mask: opcode and function bits
        0x00000020, // Expected: 'addu' function code with rs = rt = $zr
        "clear",
        [](DisassemblyData& data, uint32_t opcode) {
            uint8_t rd = (opcode >> 11) & 0x1F;

            data.mnemonic = "clear";
            data.operands.clear();
            data.operands.push_back({"$" + mips_register_names[rd], rd});
        }
    });

    // Add more pseudoinstructions as needed
}

// Function to match and handle pseudoinstructions
std::optional<std::string> Disassembler::match_pseudo_opcode(uint32_t opcode) const {
    for (const auto& pseudo : pseudo_opcodes) {
        if ((opcode & pseudo.mask) == pseudo.expected) {
            return pseudo.mnemonic;
        }
    }
    return std::nullopt;
}

// Disassemble function
DisassemblyData Disassembler::disassemble(CPU* cpu, uint32_t pc, uint32_t opcode, bool pseudos) {
    // Check for pseudoinstruction first
    if (pseudos) {
        for (const auto& pseudo : pseudo_opcodes) {
            if ((opcode & pseudo.mask) == pseudo.expected) {
                DisassemblyData data(pc, OpcodeEntry(pseudo.mnemonic, InstructionType::Unknown, false));
                pseudo.handler(data, opcode);
                return data;
            }
        }
    }

    uint8_t function = (opcode >> 26) & 0x3F;  // Main opcode function
    auto entry = opcode_mnemonics.contains(function) ? opcode_mnemonics[function] : OpcodeEntry();

    DisassemblyData data(pc, entry);

    // Handle main instruction types based on entry type
    if (entry.type != InstructionType::Unknown) {
        switch (entry.type) {
            case InstructionType::RType: {
                uint8_t rd = (opcode >> 11) & 0x1F;
                uint8_t rs = (opcode >> 21) & 0x1F;
                uint8_t rt = (opcode >> 16) & 0x1F;

                data.operands.push_back({"$" + mips_register_names[rd], rd});
                data.operands.push_back({"$" + mips_register_names[rs], rs});
                data.operands.push_back({"$" + mips_register_names[rt], rt});
                break;
            }
            case InstructionType::IType: {
                uint8_t rs = (opcode >> 21) & 0x1F;
                uint8_t rt = (opcode >> 16) & 0x1F;
                int16_t imm = opcode & 0xFFFF;

                data.operands.push_back({"$" + mips_register_names[rt], rt});
                data.operands.push_back({"$" + mips_register_names[rs], rs});
                data.operands.push_back({format("{:X}", static_cast<uint32_t>(imm)), static_cast<uint32_t>(imm)});
                break;
            }
            case InstructionType::JType: {
                uint32_t target = (pc & 0xF0000000) | ((opcode & 0x03FFFFFF) << 2);
                data.operands.push_back({format("{:X}", target), target});
                data.jump_target = target;
                break;
            }
            default:
                break;
        }
    }
    // Handle COP0-3 opcodes
    else if (function >= 0x10 && function <= 0x13) {
        uint8_t cop_function = (opcode >> 21) & 0x1F;

        // Select the correct COP opcode table based on function
        if (cop_opcode_tables.contains(function)) {
            auto& cop_table = cop_opcode_tables[function];
            if (cop_table.contains(cop_function)) {
                data = DisassemblyData(pc, cop_table[cop_function]);

                // Operand processing for COP instructions (commonly use rd and rt)
                uint8_t rd = (opcode >> 11) & 0x1F;
                uint8_t rt = (opcode >> 16) & 0x1F;
                std::string rd_name = cop0_register_names[rd].empty() ? "$" + mips_register_names[rd] : cop0_register_names[rd];

                data.operands.push_back({rd_name, rd});
                data.operands.push_back({"$" + mips_register_names[rt], rt});
            } else {
                data.mnemonic = format("UNKNOWN COP{} (rs 0x{:X})", function - 0x10, cop_function);
            }
        } else {
            data.mnemonic = format("UNKNOWN COP{} (rs 0x{:X})", function - 0x10, cop_function);
        }
    }
    // Handle extended opcodes (e.g., function == 0x00)
    else if (function == 0x00) {
        uint8_t subfunction = opcode & 0x3F;
        if (extended_opcodes.contains(subfunction)) {
            data = DisassemblyData(pc, extended_opcodes[subfunction]);

            if (subfunction != 0x0F) {
                // Operand processing for extended R-type instructions
                uint8_t rd = (opcode >> 11) & 0x1F;
                uint8_t rs = (opcode >> 21) & 0x1F;
                uint8_t rt = (opcode >> 16) & 0x1F;

                data.operands.push_back({"$" + mips_register_names[rd], rd});
                data.operands.push_back({"$" + mips_register_names[rs], rs});
                data.operands.push_back({"$" + mips_register_names[rt], rt});
            }
        } else {
            data.mnemonic = format("UNKNOWN EXT (subfunction 0x{:X})", subfunction);
        }
    }
    // Unknown opcode
    else {
        data.mnemonic = format("UNKNOWN (function 0x{:X})", function);
    }

    return data;
}

// Example implementation of get_symbol_name (could be expanded)
std::string Disassembler::get_symbol_name(uint32_t address) {
    auto it = symbol_map.find(address);
    return (it != symbol_map.end()) ? it->second : "";
}
