#pragma once

#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <cpu/cpu.hh>
#include <memory>
#include <unordered_map>
#include <vector>

class EE;

class EEJIT {
public:
    EEJIT(EE* core);
    ~EEJIT();
    void execute_opcode(std::uint32_t opcode);
    void step();
    void run();

private:
    static constexpr size_t CACHE_SIZE = 1024;
    std::unordered_map<uint32_t, CompiledBlock> block_cache;
    std::vector<uint32_t> lru_queue;
    uint64_t execution_count = 0;
    bool single_instruction_mode = false;

    CompiledBlock* compile_block(uint32_t pc, bool single_instruction);
    void link_blocks();
    void evict_oldest_block();
    CompiledBlock* find_block(uint32_t pc);
    std::tuple<bool, uint32_t, bool> generate_ir_for_opcode(uint32_t opcode, uint32_t current_pc);

    EE* core;
    bool ready{false};
    std::unique_ptr<llvm::LLVMContext> context;
    std::unique_ptr<llvm::Module> module;
    std::unique_ptr<llvm::IRBuilder<>> builder;
    std::unique_ptr<llvm::ExecutionEngine> executionEngine;

    typedef void (EEJIT::*OpcodeHandler)(std::uint32_t, uint32_t&, bool&, EE*);

    struct OpcodeHandlerEntry {
        std::unordered_map<std::uint8_t, OpcodeHandler> funct3_map;
        OpcodeHandler single_handler = nullptr;
    };

    std::unordered_map<std::uint8_t, OpcodeHandlerEntry> opcode_table;

    void initialize_opcode_table();

    void ee_jit_mfc0(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sll(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
};
