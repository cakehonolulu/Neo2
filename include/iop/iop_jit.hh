#pragma once

#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/Orc/LLJIT.h>
#include <cpu/cpu.hh>
#include <constants.hh>
#include <memory>
#include <unordered_map>
#include <vector>

#define EMIT_IOP_UPDATE_PC(core, builder, current_pc) \
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

class IOP;

class IOPJIT {
public:
    IOPJIT(IOP* core);
    ~IOPJIT();
    std::unordered_map<uint32_t, CompiledBlock> block_cache;
    void step();
    void run(Breakpoint *breakpoints);

    void execute_cycles(uint64_t cycle_limit, Breakpoint *breakpoints);
    void setup_iop_jit_primitives(std::unique_ptr<llvm::Module> &new_module);
    
    uint32_t execute_block(Breakpoint *breakpoints);
    CompiledBlock *compile_block(uint32_t start_pc, Breakpoint *breakpoints);

    RunType exec_type = RunType::Run;

    std::shared_ptr<const std::unordered_map<uint32_t, CompiledBlock>> get_block_cache() const {
        return std::make_shared<const std::unordered_map<uint32_t, CompiledBlock>>(block_cache);
    }

private:
    static constexpr size_t CACHE_SIZE = 1024;
    std::vector<uint32_t> lru_queue;
    uint64_t execution_count = 0;
    bool single_instruction_mode = false;

    void link_blocks();
    void evict_oldest_block();
    CompiledBlock* find_block(uint32_t pc);
    std::tuple<bool, uint32_t, bool> generate_ir_for_opcode(uint32_t opcode, uint32_t current_pc);

    IOP* core;
    bool ready{false};
    std::unique_ptr<llvm::LLVMContext> context;
    std::unique_ptr<llvm::Module> module;
    std::unique_ptr<llvm::IRBuilder<>> builder;
    std::unique_ptr<llvm::orc::LLJIT> lljit;

    llvm::FunctionType *iop_read8_type;
    llvm::Function *iop_read8;
    llvm::FunctionType *iop_read16_type;
    llvm::Function *iop_read16;
    llvm::FunctionType* iop_read32_type;
    llvm::Function *iop_read32;

    llvm::FunctionType *iop_write8_type;
    llvm::Function *iop_write8;
    llvm::FunctionType *iop_write16_type;
    llvm::Function *iop_write16;
    llvm::FunctionType* iop_write32_type;
    llvm::Function* iop_write32;

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
    void iop_jit_addi(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_addiu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_slti(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_bne(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_andi(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_lui(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_or(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_ori(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_jr(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_beq(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_jal(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_lw(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core);
    void iop_jit_sw(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_sb(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_mtc0(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_lb(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_addu(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_sltu(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_lh(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_and(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_slt(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_j(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_lbu(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_sra(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_sltiu(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_lhu(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_srl(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_subu(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_blez(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
    void iop_jit_bgtz(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core);
};
