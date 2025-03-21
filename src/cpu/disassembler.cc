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
    opcode_mnemonics[0x06] = OpcodeEntry("blez", InstructionType::IType, true);
    opcode_mnemonics[0x07] = OpcodeEntry("bgtz", InstructionType::IType, true);
    opcode_mnemonics[0x08] = OpcodeEntry("addi", InstructionType::IType);
    opcode_mnemonics[0x09] = OpcodeEntry("addiu", InstructionType::IType);
    opcode_mnemonics[0x0A] = OpcodeEntry("slti", InstructionType::IType);
    opcode_mnemonics[0x0B] = OpcodeEntry("sltiu", InstructionType::IType);
    opcode_mnemonics[0x0C] = OpcodeEntry("andi", InstructionType::RType);
    opcode_mnemonics[0x0D] = OpcodeEntry("ori", InstructionType::IType);
    opcode_mnemonics[0x0E] = OpcodeEntry("xori", InstructionType::IType);
    opcode_mnemonics[0x0F] = OpcodeEntry("lui", InstructionType::IType);
    opcode_mnemonics[0x14] = OpcodeEntry("beql", InstructionType::IType, true);
    opcode_mnemonics[0x15] = OpcodeEntry("bnel", InstructionType::IType, true);
    opcode_mnemonics[0x19] = OpcodeEntry("daddiu", InstructionType::IType);
    opcode_mnemonics[0x1E] = OpcodeEntry("lq", InstructionType::IType);
    opcode_mnemonics[0x1F] = OpcodeEntry("sq", InstructionType::IType);
    opcode_mnemonics[0x20] = OpcodeEntry("lb", InstructionType::IType);
    opcode_mnemonics[0x21] = OpcodeEntry("lh", InstructionType::IType);
    opcode_mnemonics[0x23] = OpcodeEntry("lw", InstructionType::IType);
    opcode_mnemonics[0x24] = OpcodeEntry("lbu", InstructionType::IType);
    opcode_mnemonics[0x25] = OpcodeEntry("lhu", InstructionType::IType);
    opcode_mnemonics[0x28] = OpcodeEntry("sb", InstructionType::IType);
    opcode_mnemonics[0x29] = OpcodeEntry("sh", InstructionType::IType);
    opcode_mnemonics[0x2B] = OpcodeEntry("sw", InstructionType::IType);
    opcode_mnemonics[0x2F] = OpcodeEntry("cache", InstructionType::IType);
    opcode_mnemonics[0x37] = OpcodeEntry("ld", InstructionType::IType);
    opcode_mnemonics[0x39] = OpcodeEntry("swc1", InstructionType::IType);
    opcode_mnemonics[0x3F] = OpcodeEntry("sd", InstructionType::IType);

    // Extended opcode table (e.g., special opcodes when function == 0x00)
    extended_opcodes[0x00] = OpcodeEntry("sll", InstructionType::RType);
    extended_opcodes[0x02] = OpcodeEntry("srl", InstructionType::RType);
    extended_opcodes[0x03] = OpcodeEntry("sra", InstructionType::RType);
    extended_opcodes[0x04] = OpcodeEntry("sllv", InstructionType::RType);
    extended_opcodes[0x07] = OpcodeEntry("srav", InstructionType::RType);
    extended_opcodes[0x08] = OpcodeEntry("jr", InstructionType::JType, true);
    extended_opcodes[0x09] = OpcodeEntry("jalr", InstructionType::JType, true);
    extended_opcodes[0x0A] = OpcodeEntry("movz", InstructionType::RType);
    extended_opcodes[0x0B] = OpcodeEntry("movn", InstructionType::RType);
    extended_opcodes[0x0C] = OpcodeEntry("syscall", InstructionType::RType);
    extended_opcodes[0x0F] = OpcodeEntry("sync", InstructionType::Unknown);
    extended_opcodes[0x10] = OpcodeEntry("mfhi", InstructionType::RType);
    extended_opcodes[0x12] = OpcodeEntry("mflo", InstructionType::RType);
    extended_opcodes[0x14] = OpcodeEntry("dsllv", InstructionType::RType);
    extended_opcodes[0x17] = OpcodeEntry("dsrav", InstructionType::RType);
    extended_opcodes[0x18] = OpcodeEntry("mult", InstructionType::RType);
    extended_opcodes[0x1A] = OpcodeEntry("div", InstructionType::RType);
    extended_opcodes[0x1B] = OpcodeEntry("divu", InstructionType::RType);
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
    extended_opcodes[0x2D] = OpcodeEntry("dmove", InstructionType::RType);
    extended_opcodes[0x38] = OpcodeEntry("dsll", InstructionType::RType);
    extended_opcodes[0x3A] = OpcodeEntry("dsrl", InstructionType::RType);
    extended_opcodes[0x3C] = OpcodeEntry("dsll32", InstructionType::RType);
    extended_opcodes[0x3F] = OpcodeEntry("dsra32", InstructionType::RType);

    // COP0 Main Instructions
    cop_opcode_tables[0x10][0x00] = OpcodeEntry("mfc0", InstructionType::IType);
    cop_opcode_tables[0x10][0x04] = OpcodeEntry("mtc0", InstructionType::IType);

    // COP0 Sub-instructions (e.g., TLB and system operations)
    cop_opcode_tables[0x10][0x10].subopcodes[0x02] = OpcodeEntry("tlbwi", InstructionType::RType);
    cop_opcode_tables[0x10][0x10].subopcodes[0x18] = OpcodeEntry("eret", InstructionType::RType);
    cop_opcode_tables[0x10][0x10].subopcodes[0x38] = OpcodeEntry("ei", InstructionType::RType);
    cop_opcode_tables[0x10][0x10].subopcodes[0x39] = OpcodeEntry("di", InstructionType::RType);

    branch_opcodes[0x00] = OpcodeEntry("bltz", InstructionType::IType, true);
    branch_opcodes[0x01] = OpcodeEntry("bgez", InstructionType::IType, true);
    branch_opcodes[0x02] = OpcodeEntry("bltzl", InstructionType::IType, true);

    mmi_opcodes[0x12] = OpcodeEntry("mflo1", InstructionType::IType);
    mmi_opcodes[0x1B] = OpcodeEntry("divu1", InstructionType::IType);
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
    else if (function == 0x01) {
        uint8_t subfunction = (opcode >> 16) & 0x1F;  // Extract the subfunction for branch type

        // Check if this is a valid branch subfunction (like bgezal, bgez, etc.)
        if (branch_opcodes.contains(subfunction)) {
            auto branch_entry = branch_opcodes[subfunction];
            data.mnemonic = branch_entry.mnemonic;

            uint8_t rs = (opcode >> 21) & 0x1F;  // Extract rs (for branch instructions)
            int16_t imm = opcode & 0xFFFF;       // Immediate value (offset)

            // Populate operands
            data.operands.push_back({"$" + mips_register_names[rs], rs});
            data.operands.push_back({format("{:X}", static_cast<uint32_t>(imm)), static_cast<uint32_t>(imm)});

            // Set jump target
            uint32_t target = pc + (imm << 2);  // PC-relative jump target
            data.jump_target = target;
        } else {
            data.mnemonic = format("UNKNOWN BRANCH (subfunction 0x{:X})", subfunction);
        }
    }
    // Handle COP0-3 opcodes
    else if (function >= 0x10 && function <= 0x13) { // COP0 to COP3
        uint8_t cop_function = (opcode >> 21) & 0x1F; // Extract primary COP function (rs field)
        uint8_t sub_opcode = opcode & 0x3F;          // Extract sub-opcode (last 6 bits)

        if (cop_opcode_tables.contains(function)) {
            auto& cop_table = cop_opcode_tables[function];

            if (cop_table.contains(cop_function)) {
                auto& entry = cop_table[cop_function];

                // Handle sub-opcodes if present
                if (entry.subopcodes.contains(sub_opcode)) {
                    auto& sub_entry = entry.subopcodes[sub_opcode];
                    data.mnemonic = sub_entry.mnemonic;
                    data.type = sub_entry.type;
                } else {
                    data.mnemonic = entry.mnemonic;
                    data.type = entry.type;
                }

                // Operand decoding
                if (cop_function == 0x00 || cop_function == 0x04) { // MFC0 or MTC0
                    uint8_t rt = (opcode >> 16) & 0x1F;
                    uint8_t rd = (opcode >> 11) & 0x1F;
                    std::string rd_name = !cop0_register_names[rd].empty()
                                                ? cop0_register_names[rd]
                                                : "$" + mips_register_names[rd];
                    data.operands.push_back({"$" + mips_register_names[rt], rt});
                    data.operands.push_back({"$" + rd_name, rd});
                }
            } else {
                data.mnemonic = format("UNKNOWN COP{} (rs 0x{:X})", function - 0x10, cop_function);
            }
        } else {
            data.mnemonic = format("UNKNOWN COP{} (rs 0x{:X})", function - 0x10, cop_function);
        }
    }
    else if (function == 0x1C) {
        uint8_t subfunction = opcode & 0x1F;  // Extract the subfunction for MMI

        // Check if this is a valid MMI subfunction
        if (mmi_opcodes.contains(subfunction)) {
            auto mmi_entry = mmi_opcodes[subfunction];
            data.mnemonic = mmi_entry.mnemonic;

            // Operand processing for MMI instructions
            uint8_t rd = (opcode >> 11) & 0x1F;
            uint8_t rs = (opcode >> 21) & 0x1F;
            uint8_t rt = (opcode >> 16) & 0x1F;

            data.operands.push_back({"$" + mips_register_names[rd], rd});
            data.operands.push_back({"$" + mips_register_names[rs], rs});
            data.operands.push_back({"$" + mips_register_names[rt], rt});
        } else {
            data.mnemonic = format("UNKNOWN MMI (subfunction 0x{:X})", subfunction);
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
