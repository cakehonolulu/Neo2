#pragma once

#include "cpu.hh"
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// Enum defining different instruction types
enum class InstructionType {
    RType,
    IType,
    JType,
    Unknown
};

// Opcode entry containing metadata for each instruction
struct OpcodeEntry {
    std::string mnemonic;
    InstructionType type;
    bool is_jump;

    OpcodeEntry(const std::string& m = "UNKNOWN", InstructionType t = InstructionType::Unknown, bool j = false)
        : mnemonic(m), type(t), is_jump(j) {}
};

// Operand structure for holding decoded operand data
struct Operand {
    std::string text;   // Operand label (e.g., "$rd")
    uint32_t value;     // Actual value of the operand
};

// Main disassembly data structure for each instruction
struct DisassemblyData {
    uint32_t pc;
    std::string mnemonic;
    std::vector<Operand> operands;
    bool is_jump;
    uint32_t jump_target;
    InstructionType type;

    DisassemblyData(uint32_t pc_, const OpcodeEntry& entry)
        : pc(pc_), mnemonic(entry.mnemonic), is_jump(entry.is_jump), jump_target(0), type(entry.type) {}
};

// Structure to define a pseudoinstruction
struct PseudoOpcode {
    uint32_t mask;        // Mask to isolate bits involved in pseudoinstruction
    uint32_t expected;    // Expected result after masking for it to qualify
    std::string mnemonic; // Pseudoinstruction mnemonic
    std::function<void(DisassemblyData&, uint32_t)> handler; // Handler to modify DisassemblyData
};

// Disassembler class with opcode table and disassemble method
class Disassembler {
public:
    Disassembler();
    DisassemblyData disassemble(CPU* cpu, uint32_t pc, uint32_t opcode, bool pseudos);
    std::string get_symbol_name(uint32_t address);

private:
    // Flag to toggle pseudoinstruction usage
    std::unordered_map<uint8_t, OpcodeEntry> opcode_mnemonics;
    std::unordered_map<uint8_t, OpcodeEntry> extended_opcodes;
    std::unordered_map<uint8_t, std::unordered_map<uint8_t, OpcodeEntry>> cop_opcode_tables;
    std::vector<PseudoOpcode> pseudo_opcodes; // List of pseudoinstructions

    void initialize_opcode_table();
    void initialize_pseudo_opcode_table(); // Initialize pseudoinstructions
    std::optional<std::string> match_pseudo_opcode(uint32_t opcode) const;
    std::unordered_map<uint32_t, std::string> symbol_map;
};
