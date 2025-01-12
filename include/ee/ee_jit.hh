#pragma once

#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <cpu/cpu.hh>
#include <memory>
#include <unordered_map>
#include <vector>

#define EMIT_EE_UPDATE_PC(core, builder, current_pc) \
    do { \
        llvm::Value* pc_ptr = builder->CreateIntToPtr( \
            builder->getInt64(reinterpret_cast<uint64_t>(&(core->pc))), \
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
        llvm::Value* new_pc; \
        llvm::Value* updated_branching = builder->getInt1(false); \
        llvm::Value* updated_pc = builder->CreateAdd(builder->getInt32(current_pc), builder->getInt32(4)); \
        new_pc = builder->CreateSelect( \
            branching, \
            branch_dest, \
            updated_pc \
        ); \
        builder->CreateStore(new_pc, pc_ptr); \
        builder->CreateStore(updated_branching, branching_ptr); \
    } while (0)

class EE;

class EEJIT {
public:
    EEJIT(EE* core);
    ~EEJIT();
    std::unordered_map<uint32_t, CompiledBlock> block_cache;
    void execute_opcode(std::uint32_t opcode);
    void step();
    void run();

private:
    static constexpr size_t CACHE_SIZE = 1024;
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

    llvm::FunctionType* ee_write32_type;
    llvm::Function* ee_write32;
    llvm::FunctionType* ee_write64_type;
    llvm::Function* ee_write64;
    llvm::FunctionType* ee_write128_type;
    llvm::Function* ee_write128;

    llvm::FunctionType* ee_read8_type;
    llvm::Function* ee_read8;
    llvm::FunctionType* ee_read32_type;
    llvm::Function* ee_read32;
    llvm::FunctionType* ee_read128_type;
    llvm::Function* ee_read128;

    typedef void (EEJIT::*OpcodeHandler)(std::uint32_t, uint32_t&, bool&, EE*);

    struct OpcodeHandlerEntry {
        std::unordered_map<std::uint8_t, OpcodeHandler> funct3_map;
        std::unordered_map<std::uint8_t, OpcodeHandler> rs_map;
        OpcodeHandler single_handler = nullptr;
    };

    std::unordered_map<std::uint8_t, OpcodeHandlerEntry> opcode_table;

    void base_error_handler(uint32_t opcode);

    void initialize_opcode_table();

    void ee_jit_mfc0(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_mtc0(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sll(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_addiu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sync(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_slti(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_bne(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_lui(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_ori(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_jr(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sw(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_srl(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_tlbwi(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_jalr(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sd(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_move(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_jal(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_andi(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_beq(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_or(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_mult(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_divu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_beql(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_mflo(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sltiu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_bnel(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_lb(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_lbu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_swc1(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sra(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_addu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_lw(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_ld(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
};
