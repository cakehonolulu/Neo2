#pragma once

#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/Orc/LLJIT.h>
#include <cpu/cpu.hh>
#include <cpu/breakpoint.hh>
#include <memory>
#include <unordered_map>
#include <vector>
#include <cpu/breakpoint.hh>
#include <log/log.hh>
#include <constants.hh>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/Verifier.h>

#define Default     9
#define Branch      11
#define CopDefault  7

#define Mult        2*8
#define Div         14*8
#define MMI_Mult    3*8
#define MMI_Div     22*8
#define MMI_Default 14

#define FPU_Mult    4*8

#define Store       14
#define Load        14

#define CAUSE_SYSCALL  (8 << 2)

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
        if (single_instruction_mode) \
        { \
            llvm::Value* likely_ptr = builder->CreateIntToPtr( \
                builder->getInt64(reinterpret_cast<uint64_t>(&(core->likely_branch))), \
                llvm::PointerType::getUnqual(builder->getInt1Ty()) \
            ); \
            llvm::Value* likely = builder->CreateLoad(builder->getInt1Ty(), likely_ptr); \
            llvm::Value* likely_branch_cond = builder->CreateICmpEQ(likely, builder->getTrue()); \
            branch_dest = builder->CreateSelect(likely_branch_cond, builder->CreateAdd(branch_dest, builder->getInt32(4)), branch_dest); \
            builder->CreateStore(builder->getFalse(), likely_ptr); \
        } \
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
    void step();
    void run(Breakpoint *breakpoints);

    uint32_t execute_block(Breakpoint *breakpoints);
    CompiledBlock* compile_block(uint32_t start_pc, Breakpoint *breakpoints);

    // New method to execute a limited number of cycles
    void execute_cycles(uint64_t cycle_limit, Breakpoint *breakpoints);

    RunType exec_type = RunType::Run;

    std::shared_ptr<const std::unordered_map<uint32_t, CompiledBlock>> get_block_cache() const {
        return std::make_shared<const std::unordered_map<uint32_t, CompiledBlock>>(block_cache);
    }

    void ee_jit_set_run_elf(bool state)
    {
        run_elf = state;
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

    EE* core;
    bool ready{false};
    std::unique_ptr<llvm::LLVMContext> context;
    std::unique_ptr<llvm::Module> module;
    std::unique_ptr<llvm::IRBuilder<>> builder;
    std::unique_ptr<llvm::orc::LLJIT> lljit;

    llvm::FunctionType* ee_write8_type;
    llvm::Function* ee_write8;
    llvm::FunctionType* ee_write16_type;
    llvm::Function* ee_write16;
    llvm::FunctionType* ee_write32_type;
    llvm::Function* ee_write32;
    llvm::FunctionType* ee_write64_type;
    llvm::Function* ee_write64;
    llvm::FunctionType* ee_write128_type;
    llvm::Function* ee_write128;

    llvm::FunctionType* ee_read8_type;
    llvm::Function* ee_read8;
    llvm::FunctionType* ee_read16_type;
    llvm::Function* ee_read16;
    llvm::FunctionType* ee_read32_type;
    llvm::Function* ee_read32;
    llvm::FunctionType* ee_read64_type;
    llvm::Function* ee_read64;
    llvm::FunctionType* ee_read128_type;
    llvm::Function* ee_read128;

    llvm::FunctionType* ee_tlb_write_type;
    llvm::Function* ee_tlb_write;

    llvm::FunctionType* ee_update_address_mapping_type;
    llvm::Function* ee_update_address_mapping;

    llvm::FunctionType* ee_write32_dbg_type;
    llvm::Function* ee_write32_dbg;

    llvm::FunctionType* ee_load_elf_type;
    llvm::Function* ee_load_elf;

    llvm::FunctionType* ee_print_syscall_type;
    llvm::Function* ee_print_syscall;

    bool run_elf = false;

    typedef void (EEJIT::*OpcodeHandler)(std::uint32_t, uint32_t&, bool&, EE*);

    struct OpcodeHandlerEntry {
        std::unordered_map<std::uint8_t, std::pair<OpcodeHandler, uint32_t>> funct3_map;
        std::unordered_map<std::uint8_t, std::unordered_map<std::uint8_t, std::pair<OpcodeHandler, uint32_t>>> subfunc_map;
        std::unordered_map<std::uint8_t, std::pair<OpcodeHandler, uint32_t>> rs_map;
        std::pair<OpcodeHandler, uint32_t> single_handler = {nullptr, 0};
    };

    std::unordered_map<std::uint8_t, OpcodeHandlerEntry> opcode_table;

    void base_error_handler(uint32_t opcode, uint32_t pc);

    void initialize_opcode_table();

    void setup_ee_jit_primitives(std::unique_ptr<llvm::Module>& new_module);

    void ee_jit_level1_exception(EE* core, uint32_t cause, uint32_t& current_pc);

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
    void ee_jit_dmove(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
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
    void ee_jit_j(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sb(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_bgez(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_div(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_mfhi(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sltu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_movn(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_blez(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_subu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_bgtz(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_slt(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_and(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_lhu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_bltz(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sh(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_divu1(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_mflo1(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_dsrav(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_dsll32(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_dsra32(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_xori(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_mult1(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_movz(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_dsllv(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_daddiu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sq(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_lq(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_dsrl(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_lh(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_cache(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sllv(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_dsll(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_srav(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_nor(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_cfc2(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_ctc2(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_lwu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_ldl(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_ldr(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sdl(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sdr(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_srlv(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_dsrl32(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_padduw(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_di(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_eret(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_syscall(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_bltzl(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_sub(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_add(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_addi(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_ei(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, EE *core);
    void ee_jit_div1(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, EE *core);
    void ee_jit_xor(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, EE *core);
    void ee_jit_mthi(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, EE *core);
    void ee_jit_mtlo(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, EE *core);
    void ee_jit_bgezl(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, EE *core);
    void ee_jit_multu(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, EE *core);

    void ee_jit_cop1(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);
    void ee_jit_cop2(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core);

    // New member to track the remaining cycles to execute
    uint64_t remaining_cycles;
};
