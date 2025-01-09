#pragma once

#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <cpu/cpu.hh>
#include <memory>
#include <unordered_map>
#include <vector>

#define EMIT_IOP_UPDATE_PC(core, builder, current_pc) \
    do { \
        llvm::Value* pc_ptr = builder->CreateIntToPtr( \
            builder->getInt64(reinterpret_cast<uint64_t>(&(core->pc))), \
            llvm::PointerType::getUnqual(builder->getInt32Ty()) \
        ); \
        llvm::Value* next_pc_ptr = builder->CreateIntToPtr( \
            builder->getInt64(reinterpret_cast<uint64_t>(&(core->next_pc))), \
            llvm::PointerType::getUnqual(builder->getInt32Ty()) \
        ); \
        llvm::Value* branching_ptr = builder->CreateIntToPtr( \
            builder->getInt64(reinterpret_cast<uint64_t>(&(core->branching))), \
            llvm::PointerType::getUnqual(builder->getInt1Ty()) \
        ); \
        llvm::Value* branch_dest_ptr = builder->CreateIntToPtr( \
            builder->getInt64(reinterpret_cast<uint64_t>(&(core->branch_dest))), \
            llvm::PointerType::getUnqual(builder->getInt32Ty()) \
        ); \
        llvm::Value* branching = builder->CreateLoad(builder->getInt1Ty(), branching_ptr); \
        llvm::Value* branch_dest = builder->CreateLoad(builder->getInt32Ty(), branch_dest_ptr); \
        llvm::Value* new_pc = builder->CreateSelect(branching, branch_dest, builder->CreateAdd(builder->getInt32(current_pc), builder->getInt32(4))); \
        builder->CreateStore(new_pc, pc_ptr); \
        builder->CreateStore(builder->CreateAdd(new_pc, builder->getInt32(4)), next_pc_ptr); \
        builder->CreateStore(builder->getInt1(false), branching_ptr); \
    } while (0)

class IOP;

class IOPJIT {
public:
    IOPJIT(IOP* core);
    ~IOPJIT();
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

    IOP* core;
    bool ready{false};
    std::unique_ptr<llvm::LLVMContext> context;
    std::unique_ptr<llvm::Module> module;
    std::unique_ptr<llvm::IRBuilder<>> builder;
    std::unique_ptr<llvm::ExecutionEngine> executionEngine;

    typedef void (IOPJIT::*OpcodeHandler)(std::uint32_t, uint32_t&, bool&, IOP*);

    struct OpcodeHandlerEntry {
        std::unordered_map<std::uint8_t, OpcodeHandler> funct3_map;
        std::unordered_map<std::uint8_t, OpcodeHandler> rs_map;
        OpcodeHandler single_handler = nullptr;
    };

    std::unordered_map<std::uint8_t, OpcodeHandlerEntry> opcode_table;

    void base_error_handler(uint32_t opcode);

    void initialize_opcode_table();

    void iop_jit_mfc0(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_sll(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_slti(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_bne(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_lui(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_ori(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_jr(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_beq(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
};
