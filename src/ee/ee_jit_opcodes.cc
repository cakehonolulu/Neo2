#include "cpu/breakpoint.hh"
#include <ee/ee_jit.hh>
#include <log/log.hh>
#include <neo2.hh>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/Verifier.h>
#include <sstream>
#include <thread>
#include <unistd.h>

void EEJIT::initialize_opcode_table() {
    opcode_table[0x00].funct3_map[0x00] = {&EEJIT::ee_jit_sll, Default}; // SLL opcode
    opcode_table[0x00].funct3_map[0x02] = {&EEJIT::ee_jit_srl, Default}; // SRL opcode
    opcode_table[0x00].funct3_map[0x03] = {&EEJIT::ee_jit_sra, Default}; // SRA opcode
    opcode_table[0x00].funct3_map[0x04] = {&EEJIT::ee_jit_sllv, Default}; // SLLV opcode
    opcode_table[0x00].funct3_map[0x06] = {&EEJIT::ee_jit_srlv, Default}; // SRLV opcode
    opcode_table[0x00].funct3_map[0x07] = {&EEJIT::ee_jit_srav, Default}; // SRAV opcode
    opcode_table[0x00].funct3_map[0x08] = {&EEJIT::ee_jit_jr, Default}; // JR opcode
    opcode_table[0x00].funct3_map[0x09] = {&EEJIT::ee_jit_jalr, Default}; // JALR opcode
    opcode_table[0x00].funct3_map[0x0A] = {&EEJIT::ee_jit_movz, Default}; // MOVZ opcode
    opcode_table[0x00].funct3_map[0x0B] = {&EEJIT::ee_jit_movn, Default}; // MOVN opcode
    opcode_table[0x00].funct3_map[0x0C] = {&EEJIT::ee_jit_syscall, Default}; // SYSCALL opcode
    opcode_table[0x00].funct3_map[0x0F] = {&EEJIT::ee_jit_sync, Default}; // SYNC opcode
    opcode_table[0x00].funct3_map[0x10] = {&EEJIT::ee_jit_mfhi, Default}; // MFHI opcode
    opcode_table[0x00].funct3_map[0x12] = {&EEJIT::ee_jit_mflo, Default}; // MFLO opcode
    opcode_table[0x00].funct3_map[0x14] = {&EEJIT::ee_jit_dsllv, Default}; // DSLLV opcode
    opcode_table[0x00].funct3_map[0x17] = {&EEJIT::ee_jit_dsrav, Default}; // DSRAV opcode
    opcode_table[0x00].funct3_map[0x18] = {&EEJIT::ee_jit_mult, Mult};  // MULT opcode
    opcode_table[0x00].funct3_map[0x1A] = {&EEJIT::ee_jit_div, Div}; // DIV opcode
    opcode_table[0x00].funct3_map[0x1B] = {&EEJIT::ee_jit_divu, Div}; // DIVU opcode
    opcode_table[0x00].funct3_map[0x20] = {&EEJIT::ee_jit_add, Default}; // ADD opcode
    opcode_table[0x00].funct3_map[0x21] = {&EEJIT::ee_jit_addu, Default}; // ADDU opcode
    opcode_table[0x00].funct3_map[0x22] = {&EEJIT::ee_jit_sub, Default}; // SUB opcode
    opcode_table[0x00].funct3_map[0x23] = {&EEJIT::ee_jit_subu, Default}; // SUBU opcode
    opcode_table[0x00].funct3_map[0x24] = {&EEJIT::ee_jit_and, Default}; // AND opcode
    opcode_table[0x00].funct3_map[0x25] = {&EEJIT::ee_jit_or, Default}; // OR opcode
    opcode_table[0x00].funct3_map[0x27] = {&EEJIT::ee_jit_nor, Default}; // NOR opcode
    opcode_table[0x00].funct3_map[0x2A] = {&EEJIT::ee_jit_slt, Default}; // SLT opcode
    opcode_table[0x00].funct3_map[0x2B] = {&EEJIT::ee_jit_sltu, Default}; // SLTU opcode
    opcode_table[0x00].funct3_map[0x2D] = {&EEJIT::ee_jit_dmove, Default}; // DMOVE opcode
    opcode_table[0x00].funct3_map[0x38] = {&EEJIT::ee_jit_dsll, Default}; // DSLL opcode
    opcode_table[0x00].funct3_map[0x3A] = {&EEJIT::ee_jit_dsrl, Default}; // DSRL opcode
    opcode_table[0x00].funct3_map[0x3C] = {&EEJIT::ee_jit_dsll32, Default}; // DSLL32 opcode
    opcode_table[0x00].funct3_map[0x3E] = {&EEJIT::ee_jit_dsrl32, Default}; // DSRL32 opcode
    opcode_table[0x00].funct3_map[0x3F] = {&EEJIT::ee_jit_dsra32, Default}; // DSRA32 opcode

    opcode_table[0x01].funct3_map[0x00] = {&EEJIT::ee_jit_bltz, Branch};  // BLTZ opcode
    opcode_table[0x01].funct3_map[0x01] = {&EEJIT::ee_jit_bgez, Branch};  // BGEZ opcode
    opcode_table[0x01].funct3_map[0x02] = {&EEJIT::ee_jit_bltzl, Branch};  // BLTZL opcode

    opcode_table[0x02].single_handler = {&EEJIT::ee_jit_j, Default}; // J opcode
    opcode_table[0x03].single_handler = {&EEJIT::ee_jit_jal, Default}; // JAL opcode
    opcode_table[0x04].single_handler = {&EEJIT::ee_jit_beq, Branch}; // BEQ opcode
    opcode_table[0x05].single_handler = {&EEJIT::ee_jit_bne, Branch}; // BNE opcode
    opcode_table[0x06].single_handler = {&EEJIT::ee_jit_blez, Branch}; // BLEZ opcode
    opcode_table[0x07].single_handler = {&EEJIT::ee_jit_bgtz, Branch}; // BGTZ opcode
    opcode_table[0x08].single_handler = {&EEJIT::ee_jit_addi, Default}; // ADDI opcode
    opcode_table[0x09].single_handler = {&EEJIT::ee_jit_addiu, Default}; // ADDIU opcode

    opcode_table[0x10].rs_map[0x00] = {&EEJIT::ee_jit_mfc0, CopDefault}; // MFC0 opcode
    opcode_table[0x10].rs_map[0x04] = {&EEJIT::ee_jit_mtc0, CopDefault}; // MTC0 opcode
    
    opcode_table[0x10].subfunc_map[0x10][0x02] = {&EEJIT::ee_jit_tlbwi, CopDefault};  // TLBWI opcode
    opcode_table[0x10].subfunc_map[0x10][0x18] = {&EEJIT::ee_jit_eret, CopDefault};  // ERET opcode
    opcode_table[0x10].subfunc_map[0x10][0x38] = {&EEJIT::ee_jit_ei, CopDefault};  // EI opcode
    opcode_table[0x10].subfunc_map[0x10][0x39] = {&EEJIT::ee_jit_di, CopDefault};  // DI opcode

    opcode_table[0x11].rs_map[0x02] = {&EEJIT::ee_jit_cop1, CopDefault}; // COP1 opcode

    opcode_table[0x12].rs_map[0x02] = {&EEJIT::ee_jit_cfc2, CopDefault}; // CFC2 opcode
    opcode_table[0x12].rs_map[0x06] = {&EEJIT::ee_jit_ctc2, CopDefault}; // CTC2 opcode

    opcode_table[0x0A].single_handler = {&EEJIT::ee_jit_slti, Default}; // SLTI opcode
    opcode_table[0x0B].single_handler = {&EEJIT::ee_jit_sltiu, Default}; // SLTIU opcode
    opcode_table[0x0C].single_handler = {&EEJIT::ee_jit_andi, Default}; // ANDI opcode
    opcode_table[0x0D].single_handler = {&EEJIT::ee_jit_ori, Default}; // ORI opcode
    opcode_table[0x0E].single_handler = {&EEJIT::ee_jit_xori, Default}; // XORI opcode
    opcode_table[0x0F].single_handler = {&EEJIT::ee_jit_lui, Default}; // LUI opcode
    opcode_table[0x14].single_handler = {&EEJIT::ee_jit_beql, Branch}; // BEQL opcode
    opcode_table[0x15].single_handler = {&EEJIT::ee_jit_bnel, Branch}; // BNEL opcode
    opcode_table[0x19].single_handler = {&EEJIT::ee_jit_daddiu, Default};  // DADDIU opcode
    opcode_table[0x1A].single_handler = {&EEJIT::ee_jit_ldl, Load};  // LDL opcode
    opcode_table[0x1B].single_handler = {&EEJIT::ee_jit_ldr, Load};  // LDR opcode

    opcode_table[0x1C].funct3_map[0x12] = {&EEJIT::ee_jit_mflo1, Default}; // MFLO1 opcode
    opcode_table[0x1C].funct3_map[0x18] = {&EEJIT::ee_jit_mult1, MMI_Mult}; // MULT1 opcode
    opcode_table[0x1C].funct3_map[0x1B] = {&EEJIT::ee_jit_divu1, MMI_Div}; // DIVU1 opcode
    opcode_table[0x1C].subfunc_map[0x28][0x10] = {&EEJIT::ee_jit_padduw, Default}; // PADDUW opcode
    opcode_table[0x1C].subfunc_map[0x29][0x12] = {&EEJIT::ee_jit_or, Default}; // POR opcode

    opcode_table[0x1E].single_handler = {&EEJIT::ee_jit_lq, Load};  // LQ opcode
    opcode_table[0x1F].single_handler = {&EEJIT::ee_jit_sq, Store};  // SQ opcode
    opcode_table[0x20].single_handler = {&EEJIT::ee_jit_lb, Load};  // LB opcode
    opcode_table[0x21].single_handler = {&EEJIT::ee_jit_lh, Load};  // LH opcode
    opcode_table[0x23].single_handler = {&EEJIT::ee_jit_lw, Load}; // LW opcode
    opcode_table[0x24].single_handler = {&EEJIT::ee_jit_lbu, Load}; // LBU opcode
    opcode_table[0x25].single_handler = {&EEJIT::ee_jit_lhu, Load}; // LHU opcode
    opcode_table[0x27].single_handler = {&EEJIT::ee_jit_lwu, Load}; // LWU opcode
    opcode_table[0x28].single_handler = {&EEJIT::ee_jit_sb, Store}; // SB opcode
    opcode_table[0x29].single_handler = {&EEJIT::ee_jit_sh, Store}; // SH opcode
    opcode_table[0x2B].single_handler = {&EEJIT::ee_jit_sw, Store}; // SW opcode
    opcode_table[0x2C].single_handler = {&EEJIT::ee_jit_sdl, Store}; // SDL opcode
    opcode_table[0x2d].single_handler = {&EEJIT::ee_jit_sdr, Store}; // SDR opcode
    opcode_table[0x2F].single_handler = {&EEJIT::ee_jit_cache, Default}; // CACHE opcode
    opcode_table[0x31].single_handler = {&EEJIT::ee_jit_cop1, Store}; // SWC1 opcode
    opcode_table[0x37].single_handler = {&EEJIT::ee_jit_ld, Load}; // LD opcode
    opcode_table[0x39].single_handler = {&EEJIT::ee_jit_swc1, Store}; // SWC1 opcode
    opcode_table[0x3F].single_handler = {&EEJIT::ee_jit_sd, Store}; // SD opcode
}

void EEJIT::ee_jit_level1_exception(EE* core, uint32_t cause, uint32_t& current_pc) {
    auto& cop0 = core->cop0;

    llvm::Value* cause_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&cop0.cause)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* vector = builder->getInt32(0x00000180); // Offset for V_COMMON

    llvm::Value* cause_ = builder->CreateLoad(builder->getInt32Ty(), cause_ptr);

    llvm::Value* mask = builder->getInt32(~0x0000007C);

    cause_ = builder->CreateAnd(cause_, mask);

    cause_ = builder->CreateOr(cause_, builder->getInt32(cause));

    builder->CreateStore(cause_, cause_ptr);

    // Prepare LLVM values for COP0 registers
    llvm::Value* status_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&cop0.Status)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* epc_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&cop0.epc)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* pc_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->pc)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Prepare branching-related LLVM values
    llvm::Value* branching_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)),
        llvm::PointerType::getUnqual(builder->getInt1Ty())
    );
    llvm::Value* branching = builder->CreateLoad(builder->getInt1Ty(), branching_ptr);

    // Determine if we are in a branch delay slot
    llvm::Value* in_branch_delay = builder->CreateICmpEQ(branching, builder->getInt1(true));

    // Load current Status and PC
    llvm::Value* status = builder->CreateLoad(builder->getInt32Ty(), status_ptr);
    llvm::Value* pc = builder->CreateLoad(builder->getInt32Ty(), pc_ptr);

    // Check if already in an exception (EXL bit set)
    llvm::Value* exl_bit = builder->CreateAnd(status, builder->getInt32(1 << 1)); // EXL is bit 1
    llvm::Value* exl_check = builder->CreateICmpNE(exl_bit, builder->getInt32(0));

    // Create blocks for exception handling
    llvm::BasicBlock* in_exception_block = llvm::BasicBlock::Create(*context, "in_exception", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* normal_exception_block = llvm::BasicBlock::Create(*context, "normal_exception", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* branch_delay_block = llvm::BasicBlock::Create(*context, "branch_delay", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* no_branch_block = llvm::BasicBlock::Create(*context, "no_branch", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* merge_block = llvm::BasicBlock::Create(*context, "merge", builder->GetInsertBlock()->getParent());

    // Handle if already in an exception
    builder->CreateCondBr(exl_check, in_exception_block, normal_exception_block);

    // In Exception Block: Use general vector (V_COMMON)
    builder->SetInsertPoint(in_exception_block);
    llvm::Value* vector_base = builder->CreateSelect(
        builder->CreateICmpNE(builder->CreateAnd(status, builder->getInt32(1 << 22)), builder->getInt32(0)), // BEV bit
        builder->getInt32(0xBFC00000), // Bootstrap base
        builder->getInt32(0x80000000)  // Normal base
    );
    llvm::Value* exception_pc = builder->CreateAdd(vector_base, vector);
    builder->CreateStore(exception_pc, pc_ptr); // Jump to exception handler
    builder->CreateBr(merge_block);

    // Normal Exception Block
    builder->SetInsertPoint(normal_exception_block);
    llvm::Value* cause_value = builder->getInt32(cause); // Exception cause code
    builder->CreateCondBr(in_branch_delay, branch_delay_block, no_branch_block);

    // Branch Delay Block: Save PC - 4 and set Cause.BD
    builder->SetInsertPoint(branch_delay_block);
    llvm::Value* pc_minus_4 = builder->CreateSub(pc, builder->getInt32(4));
    builder->CreateStore(pc_minus_4, epc_ptr);
    llvm::Value* cause_bd = builder->CreateOr(cause_value, builder->getInt32(1 << 31)); // Set Cause.BD
    builder->CreateStore(cause_bd, cause_ptr);

    llvm::Value* vector_base_bb = builder->CreateSelect(
        builder->CreateICmpNE(builder->CreateAnd(status, builder->getInt32(1 << 22)), builder->getInt32(0)), // BEV bit
        builder->getInt32(0xBFC00000), // Bootstrap base
        builder->getInt32(0x80000000)  // Normal base
    );
    llvm::Value* exception_pc_bb = builder->CreateAdd(vector_base_bb, vector);
    builder->CreateStore(exception_pc_bb, pc_ptr); // Jump to exception handler

    builder->CreateBr(merge_block);

    // No Branch Block: Save PC and clear Cause.BD
    builder->SetInsertPoint(no_branch_block);
    builder->CreateStore(pc, epc_ptr);
    builder->CreateStore(cause_value, cause_ptr);

    llvm::Value* vector_base_no_bb = builder->CreateSelect(
        builder->CreateICmpNE(builder->CreateAnd(status, builder->getInt32(1 << 22)), builder->getInt32(0)), // BEV bit
        builder->getInt32(0xBFC00000), // Bootstrap base
        builder->getInt32(0x80000000)  // Normal base
    );
    llvm::Value* exception_pc_no_bb = builder->CreateAdd(vector_base_no_bb, vector);
    builder->CreateStore(exception_pc_no_bb, pc_ptr); // Jump to exception handler

    builder->CreateBr(merge_block);

    // Merge Block: Finalize Status and jump to exception vector
    builder->SetInsertPoint(merge_block);
    llvm::Value* new_status = builder->CreateOr(status, builder->getInt32(1 << 1)); // Set EXL bit
    builder->CreateStore(new_status, status_ptr);
}

void EEJIT::ee_jit_mfc0(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;
    uint8_t rt = (opcode >> 16) & 0x1F;

    // Load the cycles value from core->cycles
    llvm::Value* cycles_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&(core->cycles))),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    llvm::Value* cycles_value = builder->CreateLoad(builder->getInt64Ty(), cycles_ptr);

    llvm::Value* offset_cycles = builder->CreateAdd(cycles_value, builder->getInt64(11));

    // Store the cycles value to cop0_registers[9]
    llvm::Value* cop0_reg9_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&(core->cop0.regs[9]))),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    builder->CreateStore(offset_cycles, cop0_reg9_ptr);

    llvm::Value* cop0_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->cop0.regs)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* cop0_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(rd)));

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* cop0_val_ze = builder->CreateZExt(cop0_value, builder->getInt64Ty());
    llvm::Value* gpr_u32_0 = builder->CreateGEP(builder->getInt64Ty(), gpr_base, {builder->getInt64(rt * 2)});
    builder->CreateStore(cop0_val_ze, gpr_u32_0);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_mtc0(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;
    uint8_t rt = (opcode >> 16) & 0x1F;

    llvm::Value* cop0_reg9_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&(core->cop0.regs[9]))),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* cop0_reg9 = builder->CreateLoad(builder->getInt32Ty(), cop0_reg9_ptr);
    llvm::Value* updated_cop0_reg9 = builder->CreateAdd(cop0_reg9, builder->getInt32(7));
    builder->CreateStore(updated_cop0_reg9, cop0_reg9_ptr);

    llvm::Value* gpr_base = builder->CreateIntToPtr( builder->getInt64(reinterpret_cast<uint64_t>(core->registers)), llvm::PointerType::getUnqual(builder->getInt32Ty()) );
    llvm::Value* gpr_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4)));
    llvm::Value* cop0_base = builder->CreateIntToPtr( builder->getInt64(reinterpret_cast<uint64_t>(core->cop0.regs)), llvm::PointerType::getUnqual(builder->getInt32Ty()) );
    llvm::Value* cop0_u32_0 = builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(rd));
    builder->CreateStore(gpr_value, cop0_u32_0);
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_addiu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;
    uint8_t rs = (opcode >> 21) & 0x1F;
    int16_t imm = static_cast<int16_t>(opcode & 0xFFFF); // Sign-extend immediate value

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    // Sign-extend the immediate value to 64 bits
    llvm::Value* imm_value = builder->CreateSExt(builder->getInt32(imm), builder->getInt64Ty());
    llvm::Value* rs_value_64 = builder->CreateSExt(rs_value, builder->getInt64Ty());

    llvm::Value* result = builder->CreateAdd(rs_value_64, imm_value);

    llvm::Value* rt_ptr = builder->CreateGEP(
        builder->getInt32Ty(),
        gpr_base,
        builder->getInt32(rt * 4)
    );

    builder->CreateStore(result, rt_ptr);
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_sll(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;
    uint8_t rt = (opcode >> 16) & 0x1F;
    uint8_t sa = (opcode >> 6) & 0x1F;

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rt_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4)));
    llvm::Value* shifted_value = builder->CreateShl(rt_value, builder->getInt32(sa));
    llvm::Value* sign_extended_value = builder->CreateSExt(shifted_value, builder->getInt64Ty());

    llvm::Value* rd_ptr = builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt32(rd * 2));
    builder->CreateStore(sign_extended_value, rd_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_slti(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;
    uint8_t rs = (opcode >> 21) & 0x1F;
    int16_t imm = static_cast<int16_t>(opcode & 0xFFFF);

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)));
    llvm::Value* imm_value = builder->CreateSExt(builder->getInt32(imm), builder->getInt32Ty());
    llvm::Value* result = builder->CreateICmpSLT(rs_value, imm_value);
    llvm::Value* result_int = builder->CreateZExt(result, builder->getInt64Ty());

    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2));
    builder->CreateStore(result_int, rt_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_beq(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract rs register
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract rt register
    int16_t immediate = static_cast<int16_t>(opcode & 0xFFFF);  // Extract immediate

    // Load the base address of general-purpose registers (GPR)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load the values from registers rs and rt
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Create the condition: rs == rt (equality comparison)
    llvm::Value* condition = builder->CreateICmpEQ(rs_value, rt_value);

    // Calculate the branch target address (address of the instruction in the branch delay slot + (immediate * 4))
    llvm::Value* branch_target = builder->CreateAdd(
        builder->getInt32(current_pc + 4),  // Address of the instruction in the branch delay slot
        builder->CreateShl(builder->getInt32(immediate), 2)  // immediate * 4
    );

    // Execute the delay slot instruction, regardless of whether the branch is taken or not
    EMIT_EE_UPDATE_PC(core, builder, current_pc);

    // If the branch is taken, update the branch destination and set branching to true
    builder->CreateStore(
        builder->CreateSelect(condition, branch_target, builder->getInt32(current_pc + 4)),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty()))
    );

    // Set the branching flag
    builder->CreateStore(
        builder->CreateSelect(condition, builder->getInt1(true), builder->getInt1(false)),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty()))
    );

    is_branch = true;
}

void EEJIT::ee_jit_bne(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract rs register
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract rt register
    int16_t immediate = static_cast<int16_t>(opcode & 0xFFFF); // Extract immediate

    // Load the value from registers rs and rt
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)));
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4)));

    // Compare rs and rt (not equal)
    llvm::Value* comparison = builder->CreateICmpNE(rs_value, rt_value);

    // Calculate the branch target address
    llvm::Value* branch_target = builder->CreateAdd(
        builder->getInt32(current_pc),
        builder->getInt32(immediate * 4 + 4) // Branch offset (immediate * 4 + 4 for the next instruction)
    );

    // Update the PC based on the comparison result (branch taken or not)
    EMIT_EE_UPDATE_PC(core, builder, current_pc);

    // If branch is taken, update the branch destination and set branching to true
    builder->CreateStore(
        builder->CreateSelect(comparison, branch_target, builder->getInt32(current_pc + 4)),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty()))
    );
    builder->CreateStore(
        builder->CreateSelect(comparison, builder->getInt1(true), builder->getInt1(false)),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty()))
    );

    is_branch = true;
}

void EEJIT::ee_jit_lui(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;
    int16_t imm = static_cast<int16_t>(opcode & 0xFFFF);

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* imm_value = builder->CreateShl(builder->getInt32(imm), builder->getInt32(16));

    // Sign-extend imm_value to 64 bits
    llvm::Value* imm_value_64 = builder->CreateSExt(imm_value, builder->getInt64Ty());

    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4));

    builder->CreateStore(imm_value_64, rt_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_ori(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;
    uint8_t rs = (opcode >> 21) & 0x1F;
    uint16_t imm = static_cast<uint16_t>(opcode & 0xFFFF);

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)));
    llvm::Value* imm_value = builder->getInt32(imm);
    llvm::Value* result = builder->CreateOr(rs_value, imm_value);

    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4));
    builder->CreateStore(result, rt_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_jr(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract rs register

    // Load the value from register rs (address to jump to)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load rs value (the jump address)
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)));

    // First, update the program counter using EMIT_EE_UPDATE_PC
    EMIT_EE_UPDATE_PC(core, builder, current_pc);

    // Now set the branch destination (jump address) after updating the PC
    builder->CreateStore(rs_value, builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty())));

    // Mark the processor as branching (set core->branching to true)
    builder->CreateStore(builder->getInt1(true), builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)),
        llvm::PointerType::getUnqual(builder->getInt1Ty())
    ));

    // Finally, set is_branch to true (this can now be done safely)
    is_branch = true;
}

void EEJIT::ee_jit_sync(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // DUMMY OPCODE
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_sw(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)
    // Handle 128-bit register access for rs (base register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty()) // Each register is 32-bit
    );
    // Load the base register value (rs) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)
    ));
    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);
    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value); 
    // Load the value to store from register rt
    llvm::Value* value_to_store = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4)
    ));
    
    // Call write32: write32(core, addr, value_to_store)
    builder->CreateCall(ee_write32, {llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
                                    addr_value, value_to_store});
    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_srl(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F; // Destination register
    uint8_t rt = (opcode >> 16) & 0x1F; // Source register
    uint8_t sa = (opcode >> 6) & 0x1F;  // Shift amount

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rt_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4)));
    llvm::Value* shifted_value = builder->CreateLShr(rt_value, builder->getInt32(sa));

    llvm::Value* rd_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rd * 4));
    builder->CreateStore(shifted_value, rd_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_tlbwi(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    llvm::Value* cop0_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->cop0.regs)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load core->cop0.regs[0] (Index register)
    llvm::Value* index = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(0)));

    // Load core->cop0.regs[5] (PageMask register)
    llvm::Value* page_mask = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(5)));

    // Load core->cop0.regs[10] (EntryHi register)
    llvm::Value* entry_hi = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(10)));

    // Load core->cop0.regs[2] (EntryLo0 register)
    llvm::Value* entry_lo0 = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(2)));

    // Load core->cop0.regs[3] (EntryLo1 register)
    llvm::Value* entry_lo1 = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(3)));

    builder->CreateCall(ee_tlb_write, {
        llvm::ConstantInt::get(builder->getInt64Ty(),
        reinterpret_cast<uint64_t>(core)),
            index,       // Index
            page_mask,   // PageMask
            entry_hi,    // EntryHi
            entry_lo0,   // EntryLo0
            entry_lo1    // EntryLo1
    });

    // Update PC
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_jalr(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract rs register
    uint8_t rd = (opcode >> 11) & 0x1F; // Extract rd register

    // Load the value from register rs (address to jump to)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load rs value (the jump address)
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)));

    // Store the return address (current_pc + 4) in register rd
    llvm::Value* return_address = builder->getInt32(current_pc + 8);
    builder->CreateStore(return_address, builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rd * 4)));

    // First, update the program counter using EMIT_EE_UPDATE_PC
    EMIT_EE_UPDATE_PC(core, builder, current_pc);

    // Now set the branch destination (jump address) after updating the PC
    builder->CreateStore(rs_value, builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty())));

    // Mark the processor as branching (set core->branching to true)
    builder->CreateStore(builder->getInt1(true), builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty())));

    // Finally, set is_branch to true (this can now be done safely)
    is_branch = true;
}

void EEJIT::ee_jit_sd(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Offset (16-bit immediate)

    // Base address of GPRs (64-bit values)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty()) // GPRs are 64-bit
    );

    // Load base register value (rs)
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rs * 2))
    );

    // Sign-extend the offset to 64 bits for address calculation
    llvm::Value* offset_value = builder->CreateSExt(
        builder->getInt32(offset), builder->getInt64Ty()
    );

    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);

    // Load the value from register rt (64-bit value)
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2))
    );

    // Perform a 64-bit write using the ee_write64 function
    builder->CreateCall(ee_write64, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)), 
        addr_value, 
        rt_value
    });

    // Emit PC update after executing the instruction
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_dmove(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;
    uint8_t rs = (opcode >> 21) & 0x1F;

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty()) // Note: 64-bit pointers
    );

    llvm::Value* rs_ptr = builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rs * 2));
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt64Ty(), rs_ptr);

    llvm::Value* rd_ptr = builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2));
    builder->CreateStore(rs_value, rd_ptr);

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_jal(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract the 26-bit target address from the opcode
    uint32_t target = opcode & 0x03FFFFFF;

    // The jump address is formed by shifting the target address left by 2 (to account for word alignment) 
    // and setting the upper 4 bits of the address (from the current PC)
    uint32_t jump_address = (current_pc & 0xF0000000) | (target << 2);

    // Store the return address (current_pc + 4) into the $ra register (register 31)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty()) // Each register is 32-bit
    );
    llvm::Value* return_address = builder->getInt32(current_pc + 8);
    builder->CreateStore(return_address, builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(31 * 4) // Store in $ra (register 31)
    ));

    // Call EMIT_EE_UPDATE_PC to update the program counter (pc = jump_address)
    EMIT_EE_UPDATE_PC(core, builder, current_pc);

    // Update the program counter to the jump address
    builder->CreateStore(builder->getInt32(jump_address), 
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty()))
    );
    builder->CreateStore(builder->getInt1(true), 
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty()))
    );

    is_branch = true;
}

void EEJIT::ee_jit_andi(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract registers and immediate from the opcode
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint16_t imm = opcode & 0xFFFF;  // Extract the immediate value (bits 0-15)

    // Create pointers to the general-purpose registers (GPR)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );

    // Load the value from the RS register
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rs * 2))  // 8 bytes offset for 64-bit registers
    );

    // Zero-extend the immediate value to 64 bits
    llvm::Value* imm_value = builder->getInt64(imm);

    // Perform the bitwise AND operation
    llvm::Value* result = builder->CreateAnd(rs_value, imm_value);

    // Store the result in the RT register
    llvm::Value* rt_ptr = builder->CreateGEP(
        builder->getInt64Ty(),
        gpr_base,
        builder->getInt64(rt * 2)
    );
    builder->CreateStore(result, rt_ptr);

    // Emit the program counter update
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_or(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract registers from opcode
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)

    // Create pointers to the general-purpose registers (GPR)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load the value from the RS register
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    // Load the value from the RT register
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Perform the bitwise OR operation
    llvm::Value* result = builder->CreateOr(rs_value, rt_value);

    // Store the result in the RD register
    llvm::Value* rd_ptr = builder->CreateGEP(
        builder->getInt32Ty(),
        gpr_base,
        builder->getInt32(rd * 4)
    );
    builder->CreateStore(result, rd_ptr);

    // Emit the program counter update
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_mult(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)

    // Load values from registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty()) // Note: 32-bit pointers for individual registers
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Sign-extend to 64 bits
    llvm::Value* rs_value_64 = builder->CreateSExt(rs_value, builder->getInt64Ty());
    llvm::Value* rt_value_64 = builder->CreateSExt(rt_value, builder->getInt64Ty());

    // Perform 64-bit signed multiplication
    llvm::Value* product = builder->CreateMul(rs_value_64, rt_value_64);

    // Extract the low 32 bits and store them in LO
    llvm::Value* lo_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->lo)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    llvm::Value* lo_value = builder->CreateTrunc(product, builder->getInt32Ty());
    builder->CreateStore(lo_value, lo_ptr);

    // Extract the high 32 bits and store them in HI
    llvm::Value* hi_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->hi)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    llvm::Value* hi_value = builder->CreateTrunc(builder->CreateLShr(product, builder->getInt64(32)), builder->getInt32Ty());
    builder->CreateStore(hi_value, hi_ptr);

    // Conditionally store the low 32 bits of the result in GPR[rd] if rd is not zero
    llvm::Value* zero = builder->getInt32(0);
    llvm::Value* rd_not_zero = builder->CreateICmpNE(builder->getInt32(rd), zero);
    
    llvm::Value* rd_ptr = builder->CreateGEP(
        builder->getInt32Ty(),
        gpr_base,
        builder->getInt32(rd * 4)
    );
    llvm::Value* select_value = builder->CreateSelect(rd_not_zero, lo_value, builder->getInt32(0));
    builder->CreateStore(select_value, rd_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_divu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)

    // Load values from registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Check if the divisor (rt_value) is zero
    llvm::Value* is_zero = builder->CreateICmpEQ(rt_value, builder->getInt32(0));
    llvm::Function* func = builder->GetInsertBlock()->getParent();
    llvm::BasicBlock* div_block = llvm::BasicBlock::Create(builder->getContext(), "div", func);
    llvm::BasicBlock* end_block = llvm::BasicBlock::Create(builder->getContext(), "end", func);
    builder->CreateCondBr(is_zero, end_block, div_block);

    // If divisor is not zero, perform the division
    builder->SetInsertPoint(div_block);
    llvm::Value* quotient = builder->CreateUDiv(rs_value, rt_value);
    llvm::Value* remainder = builder->CreateURem(rs_value, rt_value);

    // Store the quotient in LO register (lo)
    llvm::Value* lo_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->lo)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    builder->CreateStore(quotient, lo_ptr);

    // Store the remainder in HI register (hi)
    llvm::Value* hi_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->hi)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    builder->CreateStore(remainder, hi_ptr);
    builder->CreateBr(end_block);

    // Set insertion point to end block
    builder->SetInsertPoint(end_block);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_beql(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract rs register
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract rt register
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);  // Extract and sign-extend offset

    is_branch = true;

    // Load GPR base pointer
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load rs and rt values
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );
    
    // Load rs and rt values
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );
    
    // Equality comparison (rs == rt)
    llvm::Value* condition = builder->CreateICmpEQ(rs_value, rt_value);

    // Compute the branch destination: PC + (offset * 4)
    llvm::Value* branch_offset = builder->CreateSExt(builder->getInt32(offset), builder->getInt64Ty());
    llvm::Value* branch_dest = builder->CreateAdd(
        builder->getInt64(current_pc),
        builder->CreateMul(branch_offset, builder->getInt64(4))
    );

    // Update PC based on condition
    llvm::Value* pc_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->pc)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );

    llvm::Value* pc_new = builder->CreateSelect(
        condition,
        builder->CreateAdd(builder->getInt64(current_pc), builder->getInt64(4)),
        builder->CreateAdd(builder->getInt64(current_pc), builder->getInt64(8))
    );

    builder->CreateStore(pc_new, pc_ptr);

    // Update branching metadata
    llvm::Value* branching_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)),
        llvm::PointerType::getUnqual(builder->getInt1Ty())
    );

    llvm::Value* branching_new = builder->CreateSelect(
        condition,
        builder->getInt1(true),
        builder->getInt1(false)
    );

    builder->CreateStore(branching_new, branching_ptr);

    llvm::Value* branch_dest_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    builder->CreateStore(branch_dest, branch_dest_ptr);

    llvm::Value* likely_ptr = builder->CreateIntToPtr( \
                    builder->getInt64(reinterpret_cast<uint64_t>(&(core->likely_branch))),
                    llvm::PointerType::getUnqual(builder->getInt1Ty())
                );
                llvm::Value* likely = builder->CreateLoad(builder->getInt1Ty(), likely_ptr);
    builder->CreateStore(builder->getTrue(), likely_ptr);
}

void EEJIT::ee_jit_mflo(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)

    // Load the lower 64 bits from the LO register
    llvm::Value* lo_value_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->lo)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );

    llvm::Value* lo_value = builder->CreateLoad(builder->getInt64Ty(), lo_value_ptr);

    // Load the base address for the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );

    // Store the LO value into the destination register (RD)
    llvm::Value* rd_ptr = builder->CreateGEP(
        builder->getInt64Ty(),
        gpr_base,
        builder->getInt64(rd * 2)
    );
    builder->CreateStore(lo_value, rd_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);  // Update PC
}

void EEJIT::ee_jit_sltiu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)
    uint16_t immediate = opcode & 0xFFFF;  // Extract the immediate (bits 0-15)

    // Sign-extend immediate to 32 bits (SLTIU uses unsigned comparison)
    llvm::Value* imm_value = builder->getInt32(static_cast<uint32_t>(immediate));

    // Load the value from register GPR[rs]
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    // Perform unsigned comparison: GPR[rs] < immediate
    llvm::Value* condition = builder->CreateICmpULT(rs_value, imm_value);

    // Set GPR[rt] to 1 if condition is true, otherwise set to 0
    llvm::Value* result = builder->CreateSelect(condition, builder->getInt32(1), builder->getInt32(0));

    // Store the result into GPR[rt]
    builder->CreateStore(
        result,
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_bnel(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract rs register
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract rt register
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);  // Extract and sign-extend offset

    is_branch = true;

    // Load GPR base pointer
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load rs and rt values
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );
    
    // Load rs and rt values
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );
    
    // Equality comparison (rs == rt)
    llvm::Value* condition = builder->CreateICmpNE(rs_value, rt_value);

    // Compute the branch destination: PC + (offset * 4)
    llvm::Value* branch_offset = builder->CreateSExt(builder->getInt32(offset), builder->getInt64Ty());
    llvm::Value* branch_dest = builder->CreateAdd(
        builder->getInt64(current_pc),
        builder->CreateMul(branch_offset, builder->getInt64(4))
    );

    // Update PC based on condition
    llvm::Value* pc_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->pc)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );

    llvm::Value* pc_new = builder->CreateSelect(
        condition,
        builder->CreateAdd(builder->getInt64(current_pc), builder->getInt64(4)),
        builder->CreateAdd(builder->getInt64(current_pc), builder->getInt64(8))
    );

    builder->CreateStore(pc_new, pc_ptr);

    // Update branching metadata
    llvm::Value* branching_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)),
        llvm::PointerType::getUnqual(builder->getInt1Ty())
    );

    llvm::Value* branching_new = builder->CreateSelect(
        condition,
        builder->getInt1(true),
        builder->getInt1(false)
    );

    builder->CreateStore(branching_new, branching_ptr);

    llvm::Value* branch_dest_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    builder->CreateStore(branch_dest, branch_dest_ptr);

    llvm::Value* likely_ptr = builder->CreateIntToPtr( \
                    builder->getInt64(reinterpret_cast<uint64_t>(&(core->likely_branch))),
                    llvm::PointerType::getUnqual(builder->getInt1Ty())
                );
    builder->CreateStore(builder->getTrue(), likely_ptr);
}

void EEJIT::ee_jit_lb(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the destination register (rt)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)

    // Handle 128-bit register access for rs (base register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // Each register is 8-bit
    );
    
    // Load the base register value (rs) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4) // GPR uses 32-bit (4 bytes) offset
    ));
    
    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);
    
    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);

    // Call read8: read8(core, addr)
    llvm::Value* value_to_load = builder->CreateCall(ee_read8, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)), 
        addr_value
    });

    // Cast the loaded value to 8 bits to ensure correct zero-extension
    llvm::Value* byte_value = builder->CreateTrunc(value_to_load, builder->getInt8Ty());

    // Zero-extend the fetched byte
    llvm::Value* sign_extended_value = builder->CreateSExt(byte_value, builder->getInt64Ty());

    // Store the zero-extended value in the rt register
    llvm::Value* rt_ptr = builder->CreateGEP(
        builder->getInt64Ty(),
        gpr_base,
        builder->getInt32(rt * 2)
    );
    builder->CreateStore(sign_extended_value, rt_ptr);

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_lbu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t base = (opcode >> 21) & 0x1F;  // Extract BASE (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;    // Extract RT (bits 16-20)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);  // Extract OFFSET (bits 0-15)

    // Load the base register value
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())  // 32-bit pointers for individual registers
    );
    llvm::Value* base_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(base * 4))
    );

    // Calculate the effective address
    llvm::Value* offset_value = builder->getInt32(offset);
    llvm::Value* addr_value = builder->CreateAdd(base_value, offset_value);

    // Call read8 to fetch the byte from memory
    llvm::Value* value_to_load = builder->CreateCall(ee_read8, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)), 
        addr_value
    });

    // Cast the loaded value to 8 bits to ensure correct zero-extension
    llvm::Value* byte_value = builder->CreateTrunc(value_to_load, builder->getInt8Ty());

    // Zero-extend the fetched byte
    llvm::Value* zero_extended_value = builder->CreateZExt(byte_value, builder->getInt64Ty());

    // Store the zero-extended value in the rt register
    llvm::Value* rt_ptr = builder->CreateGEP(
        builder->getInt64Ty(),
        gpr_base,
        builder->getInt32(rt * 2)
    );
    builder->CreateStore(zero_extended_value, rt_ptr);

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_swc1(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t ft = (opcode >> 16) & 0x1F;  // Extract FT (bits 16-20)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);  // Extract the offset (16-bit immediate)

    // Handle 128-bit register access for RS (base register)
    llvm::Value* fpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->fpr)),
        llvm::PointerType::getUnqual(builder->getInt32Ty()) // Each FPU register is 32-bit
    );

    // Load the base register value (RS) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), fpr_base, builder->getInt32(rs * 4)
    ));

    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);
    
    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value); 
    
    // Load the value to store from FPR register FT
    llvm::Value* value_to_store = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), fpr_base, builder->getInt32(ft)
    ));

    // Call write32: write32(core, addr, value_to_store)
    builder->CreateCall(ee_write32, {llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
                                    addr_value, value_to_store});
    
    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_sra(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)
    uint8_t sa = (opcode >> 6) & 0x1F;   // Extract SA (bits 6-10)

    // Load the rt register value
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())  // 32-bit pointers for individual registers
    );
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Perform arithmetic right shift
    llvm::Value* shift_amount = builder->getInt32(sa);
    llvm::Value* shifted_value = builder->CreateAShr(rt_value, shift_amount);

    // Sign-extend the result
    llvm::Value* sign_extended_value = builder->CreateSExt(shifted_value, builder->getInt64Ty());

    // Store the result in the rd register
    llvm::Value* rd_ptr = builder->CreateGEP(
        builder->getInt32Ty(),
        gpr_base,
        builder->getInt32(rd * 4)
    );
    builder->CreateStore(sign_extended_value, rd_ptr);

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_addu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F; // Extract destination register (rd)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract first source register (rs)
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract second source register (rt)

    // Handle the GPR array
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty()) // Each register is 32-bit
    );

    // Load the values from the source registers (rs and rt)
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)
    ));

    llvm::Value* rt_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4)
    ));

    // Perform the addition (unsigned addition is the same as signed addition)
    llvm::Value* result = builder->CreateAdd(rs_value, rt_value);

    // Sign-extend imm_value to 64 bits
    llvm::Value* result_se_64 = builder->CreateSExt(result, builder->getInt64Ty());

    // Store the result into the destination register (rd)
    builder->CreateStore(result_se_64, builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt32(rd * 2)
    ));

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_lw(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)
    
    // Handle 128-bit register access for rs (base register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // Each register is 8-bit
    );
    
    // Load the base register value (rs) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4) // GPR uses 32-bit (4 bytes) offset
    ));
    
    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);
    
    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);
    
    // Call read32: read32(core, addr)
    llvm::Value* value_to_load = builder->CreateCall(ee_read32, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
        addr_value
    });


    // Sign-extend imm_value to 64 bits
    llvm::Value* result_se_64 = builder->CreateSExt(value_to_load, builder->getInt64Ty());

    // Store the loaded value into register rt
    builder->CreateStore(result_se_64, builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt32(rt * 2)
    ));
    
    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_ld(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);  // Extract the offset (16-bit immediate)

    // Handle 128-bit register access for rs (base register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // Each register is 8-bit for LD
    );
    
    // Load the base register value (rs) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt64Ty(), builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt64(rs * 2) // GPR uses 32-bit (4 bytes)
    ));
    
    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);
    
    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);
    
    // Call read128: read128(core, addr)
    llvm::Value* value_to_load = builder->CreateCall(ee_read64, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
        addr_value
    });

    // Store the loaded value into register rt (64-bit double register)
    builder->CreateStore(value_to_load, builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt32(rt * 2) // GPR uses 128-bit registers (2 * 64-bit)
    ));
    
    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_j(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract the 26-bit target address from the opcode
    uint32_t target = opcode & 0x03FFFFFF;  // Mask out the upper 6 bits, leaving the target address

    // Shift the target left by 2 bits to align it with the current program counter
    uint32_t target_address = (target << 2) | (current_pc & 0xF0000000);  // Set the upper bits from current PC

    // First, update the program counter using EMIT_EE_UPDATE_PC
    EMIT_EE_UPDATE_PC(core, builder, current_pc);

    // Set the branch destination to the target address (jump destination)
    builder->CreateStore(
        builder->getInt32(target_address),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty()))
    );

    // Mark the processor as branching
    builder->CreateStore(
        builder->getInt1(true),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty()))
    );

    // Finally, set is_branch to true (this can now be done safely)
    is_branch = true;
}

void EEJIT::ee_jit_sb(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the source register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the immediate (offset)

    // Handle 128-bit register access for rs (base register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty()) // Each register is 32-bit
    );

    // Load the base register value (rs) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)
    ));

    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);

    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);

    // Load the value to store from register rt
    llvm::Value* value_to_store = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4)
    ));

    // Call ee_write8 with the correct arguments: write8(core, addr, value_to_store)
    builder->CreateCall(ee_write8, {llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
                                    addr_value, value_to_store});

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_bgez(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract rs register
    int16_t immediate = static_cast<int16_t>(opcode & 0xFFFF); // Extract immediate

    // Load the value from registers rs
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)));

    // Compare rs with zero (greater than or equal to zero)
    llvm::Value* comparison = builder->CreateICmpSGE(rs_value, builder->getInt32(0));

    // Calculate the branch target address
    llvm::Value* branch_target = builder->CreateAdd(
        builder->getInt32(current_pc),
        builder->getInt32(immediate * 4 + 4) // Branch offset (immediate * 4 + 4 for the next instruction)
    );

    // Update the PC based on the comparison result (branch taken or not)
    EMIT_EE_UPDATE_PC(core, builder, current_pc);

    // If branch is taken, update the branch destination and set branching to true
    builder->CreateStore(
        builder->CreateSelect(comparison, branch_target, builder->getInt32(current_pc + 4)),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty()))
    );
    builder->CreateStore(
        builder->CreateSelect(comparison, builder->getInt1(true), builder->getInt1(false)),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty()))
    );

    is_branch = true;
}

void EEJIT::ee_jit_div(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)

    // Load values from registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Perform signed division
    llvm::Value* quotient = builder->CreateSDiv(rs_value, rt_value);  // Signed division
    llvm::Value* remainder = builder->CreateSRem(rs_value, rt_value); // Signed remainder

    // Store the quotient in LO register (lo)
    llvm::Value* lo_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->lo)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    builder->CreateStore(quotient, lo_ptr);

    // Store the remainder in HI register (hi)
    llvm::Value* hi_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->hi)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    builder->CreateStore(remainder, hi_ptr);

    // Update PC and set is_branch to false
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_mfhi(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)

    // Load the lower 64 bits from the HI register
    llvm::Value* hi_value_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->hi)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );

    llvm::Value* hi_value = builder->CreateLoad(builder->getInt64Ty(), hi_value_ptr);

    // Load the base address for the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );

    // Store the HI value into the destination register (RD)
    llvm::Value* rd_ptr = builder->CreateGEP(
        builder->getInt64Ty(),
        gpr_base,
        builder->getInt64(rd * 2)
    );
    builder->CreateStore(hi_value, rd_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_sltu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)

    // Load the value from register GPR[rs]
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rs * 2))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2))
    );

    // Perform unsigned comparison: GPR[rs] < immediate
    llvm::Value* condition = builder->CreateICmpULT(rs_value, rt_value);

    // Set GPR[rt] to 1 if condition is true, otherwise set to 0
    llvm::Value* result = builder->CreateSelect(condition, builder->getInt64(1), builder->getInt64(0));

    // Store the result into GPR[rt]
    builder->CreateStore(
        result,
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2))
    );

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_movn(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract the registers from the opcode
    uint32_t rs = (opcode >> 21) & 0x1F;  // Bits 21-25
    uint32_t rt = (opcode >> 16) & 0x1F;  // Bits 16-20
    uint32_t rd = (opcode >> 11) & 0x1F;  // Bits 11-15

    // Load the base address of the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load the values from the registers rs and rt
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rs * 2))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2))
    );

    // Check if rt is not zero (MOVN condition)
    llvm::Value* condition = builder->CreateICmpNE(rt_value, llvm::ConstantInt::get(builder->getInt64Ty(), 0));

    // Only store the value into rd if rt is not zero
    llvm::BasicBlock* store_block = llvm::BasicBlock::Create(*context, "store_block", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* after_store_block = llvm::BasicBlock::Create(*context, "after_store_block", builder->GetInsertBlock()->getParent());

    // Conditional branch based on rt != 0
    builder->CreateCondBr(condition, store_block, after_store_block);

    // Store operation block
    builder->SetInsertPoint(store_block);
    builder->CreateStore(rs_value, builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2)));

    // Continue after the store block
    builder->CreateBr(after_store_block);

    // After store block (if condition was not met, control will jump here)
    builder->SetInsertPoint(after_store_block);

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_blez(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    int16_t immediate = static_cast<int16_t>(opcode & 0xFFFF);  // Extract signed immediate (bits 0-15)

    // Load the value from register GPR[rs]
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    // Compare GPR[rs] with 0
    llvm::Value* condition = builder->CreateICmpSLE(rs_value, builder->getInt32(0));  // Check if rs_value <= 0

    // Calculate the branch target address
    llvm::Value* branch_target = builder->CreateAdd(
        builder->getInt32(current_pc),
        builder->getInt32(immediate * 4 + 4) // Branch offset (immediate * 4 + 4 for the next instruction)
    );

    // Update the PC based on the comparison result (branch taken or not)
    EMIT_EE_UPDATE_PC(core, builder, current_pc);

    // If branch is taken, update the branch destination and set branching to true
    builder->CreateStore(
        builder->CreateSelect(condition, branch_target, builder->getInt32(current_pc + 4)),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty()))
    );
    builder->CreateStore(
        builder->CreateSelect(condition, builder->getInt1(true), builder->getInt1(false)),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty()))
    );

    is_branch = true;
}

void EEJIT::ee_jit_subu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)

    // Load values from registers GPR[rs] and GPR[rt]
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Perform the subtraction (no overflow check)
    llvm::Value* result = builder->CreateSub(rs_value, rt_value);

    llvm::Value* res_se = builder->CreateSExt(result, builder->getInt64Ty());

    // Store the result into GPR[rd]
    llvm::Value* rd_ptr = builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2));
    builder->CreateStore(res_se, rd_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_bgtz(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    int16_t immediate = static_cast<int16_t>(opcode & 0xFFFF);  // Extract signed immediate (bits 0-15)

    // Load the value from register GPR[rs]
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    // Compare GPR[rs] with 0 (check if GPR[rs] > 0)
    llvm::Value* condition = builder->CreateICmpSGT(rs_value, builder->getInt32(0));  // Check if rs_value > 0

    // Calculate the branch target address
    llvm::Value* branch_target = builder->CreateAdd(
        builder->getInt32(current_pc),
        builder->getInt32(immediate * 4 + 4) // Branch offset (immediate * 4 + 4 for the next instruction)
    );

    // Update the PC based on the comparison result (branch taken or not)
    EMIT_EE_UPDATE_PC(core, builder, current_pc);

    // If branch is taken, update the branch destination and set branching to true
    builder->CreateStore(
        builder->CreateSelect(condition, branch_target, builder->getInt32(current_pc + 4)),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty()))
    );
    builder->CreateStore(
        builder->CreateSelect(condition, builder->getInt1(true), builder->getInt1(false)),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty()))
    );

    is_branch = true;
}

void EEJIT::ee_jit_slt(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)

    // Load the value from register GPR[rs]
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    // Load the value from register GPR[rt]
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Perform signed comparison: GPR[rs] < GPR[rt]
    llvm::Value* condition = builder->CreateICmpSLT(rs_value, rt_value);

    // Set GPR[rd] to 1 if condition is true, otherwise set to 0
    llvm::Value* result = builder->CreateSelect(condition, builder->getInt32(1), builder->getInt32(0));

    // Store the result into GPR[rd]
    builder->CreateStore(
        result,
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rd * 4))
    );

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_and(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Decode opcode fields
    uint8_t rs = (opcode >> 21) & 0x1F;  // Source register (rs)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Source register (rt)
    uint8_t rd = (opcode >> 11) & 0x1F;  // Destination register (rd)

    // Get the base pointer to the GPR registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty()) // GPR base pointer
    );

    // Load the 64-bit value from GPR[rs]
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt32(rs * 2)) // Offset for 64-bit values
    );

    // Load the 64-bit value from GPR[rt]
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt32(rt * 2)) // Offset for 64-bit values
    );

    // Perform the bitwise AND operation: GPR[rs] AND GPR[rt]
    llvm::Value* result = builder->CreateAnd(rs_value, rt_value);

    // Store the result into GPR[rd]
    builder->CreateStore(
        result,
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt32(rd * 2)) // Offset for 64-bit values
    );

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_lhu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t base = (opcode >> 21) & 0x1F;  // Extract BASE (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;    // Extract RT (bits 16-20)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);  // Extract OFFSET (bits 0-15)

    // Load the base register value
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())  // 32-bit pointers for individual registers
    );
    llvm::Value* base_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(base * 4))
    );

    // Calculate the effective address
    llvm::Value* offset_value = builder->getInt32(offset);
    llvm::Value* addr_value = builder->CreateAdd(base_value, offset_value);

    // Call read8 to fetch the byte from memory
    llvm::Value* value_to_load = builder->CreateCall(ee_read16, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)), 
        addr_value
    });

    // Cast the loaded value to 8 bits to ensure correct zero-extension
    llvm::Value* byte_value = builder->CreateTrunc(value_to_load, builder->getInt16Ty());

    // Zero-extend the fetched byte
    llvm::Value* zero_extended_value = builder->CreateZExt(byte_value, builder->getInt64Ty());

    // Store the zero-extended value in the rt register
    llvm::Value* rt_ptr = builder->CreateGEP(
        builder->getInt64Ty(),
        gpr_base,
        builder->getInt32(rt * 2)
    );
    builder->CreateStore(zero_extended_value, rt_ptr);

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_bltz(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);  // Extract and sign-extend offset (bits 0-15)

    // Load the base address of the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load the value from the rs register
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    // Create the condition: check if GPR[rs] is less than 0 (signed comparison)
    llvm::Value* condition = builder->CreateICmpSLT(rs_value, builder->getInt32(0));

    // Calculate the target address (current PC + (offset * 4))
    llvm::Value* branch_target = builder->CreateAdd(
        builder->getInt32(current_pc),
        builder->getInt32(offset * 4 + 4)  // Branch offset (immediate * 4 + 4 for the next instruction)
    );

    // Update the program counter after the instruction is executed (branching logic)
    EMIT_EE_UPDATE_PC(core, builder, current_pc);

    // Store the branch destination address in core->branch_dest
    builder->CreateStore(
        builder->CreateSelect(condition, branch_target, builder->getInt32(current_pc + 4)),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty()))
    );

    // Store the branching flag (true if condition met, false otherwise)
    builder->CreateStore(
        builder->CreateSelect(condition, builder->getInt1(true), builder->getInt1(false)),
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty()))
    );

    is_branch = true;
}

void EEJIT::ee_jit_sh(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the source register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the immediate (offset)

    // Handle 128-bit register access for rs (base register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty()) // Each register is 32-bit
    );

    // Load the base register value (rs) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)
    ));

    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);

    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);

    // Load the value to store from register rt
    llvm::Value* value_to_store = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4)
    ));

    // Call ee_write16 with the correct arguments: write16(core, addr, value_to_store)
    builder->CreateCall(ee_write16, {llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
                                    addr_value, value_to_store});

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_divu1(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)

    // Load values from registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Perform unsigned division
    llvm::Value* quotient = builder->CreateUDiv(rs_value, rt_value);
    llvm::Value* remainder = builder->CreateURem(rs_value, rt_value);

    // Store the quotient in LO1 register (lo1)
    llvm::Value* lo1_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->lo) + 8), // High 64 bits offset by 8 bytes
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    builder->CreateStore(quotient, lo1_ptr);

    // Store the remainder in HI1 register (hi1)
    llvm::Value* hi1_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->hi) + 8), // High 64 bits offset by 8 bytes
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    builder->CreateStore(remainder, hi1_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_mflo1(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)

    // Load LO1 value
    llvm::Value* lo1_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->lo) + 8), // High 64 bits offset by 8 bytes
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* lo1_value = builder->CreateLoad(builder->getInt32Ty(), lo1_ptr);

    // Store LO1 value in RD register
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* rd_ptr = builder->CreateGEP(
        builder->getInt32Ty(),
        gpr_base,
        builder->getInt32(rd * 4)
    );
    builder->CreateStore(lo1_value, rd_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_dsrav(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract the registers from the opcode
    uint32_t rs = (opcode >> 21) & 0x1F;  // Bits 21-25
    uint32_t rt = (opcode >> 16) & 0x1F;  // Bits 16-20
    uint32_t rd = (opcode >> 11) & 0x1F;  // Bits 11-15

    // Load the base address of the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty()) // 64-bit signed integers
    );

    // Load the values from the registers rs and rt
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2)) // Access rt register
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rs * 2)) // Access rs register
    );

    // Mask the shift amount to ensure it's within the range 0-63 (lower 6 bits)
    llvm::Value* shift_amount = builder->CreateAnd(rs_value, llvm::ConstantInt::get(builder->getInt64Ty(), 0x3F)); // Mask to lower 6 bits

    // Perform the arithmetic right shift (signed shift)
    llvm::Value* shifted_value = builder->CreateAShr(rt_value, shift_amount); // Arithmetic right shift

    // Store the result in the rd register (signed result of the shift)
    builder->CreateStore(shifted_value, builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2)));

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_dsll32(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract the registers and shift amount from the opcode
    uint32_t rt = (opcode >> 16) & 0x1F;  // Bits 16-20
    uint32_t rd = (opcode >> 11) & 0x1F;  // Bits 11-15
    uint32_t sa = (opcode >> 6) & 0x1F;   // Bits 6-10 (shift amount)
    
    // Compute the total shift amount: sa + 32
    uint32_t total_shift = 32 + sa;
    
    // Load the base address of the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty()) // 64-bit integers (doubleword)
    );

    // Load the value from the rt register
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt64Ty(), // Load as 64-bit integer
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2)) // rt * 2 for 64-bit offset
    );

    // Perform the logical left shift (shift by sa + 32)
    llvm::Value* shifted_value = builder->CreateShl(rt_value, builder->getInt32(total_shift));  // Logical shift left (64-bit shift)

    // Store the result in the rd register
    builder->CreateStore(
        shifted_value,
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2)) // rd * 2 for 64-bit offset
    );

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_dsra32(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract the registers and shift amount from the opcode
    uint32_t rt = (opcode >> 16) & 0x1F;  // Bits 16-20
    uint32_t rd = (opcode >> 11) & 0x1F;  // Bits 11-15
    uint32_t sa = (opcode >> 6) & 0x1F;   // Bits 6-10 (shift amount)
    
    // Compute the total shift amount: sa + 32
    uint32_t total_shift = 32 + sa;
    
    // Load the base address of the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())  // Use 64-bit for doubleword register
    );

    // Load the value from the rt register (64-bit)
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2))  // 64-bit offset for rt
    );

    // Perform the arithmetic right shift (shift by sa + 32)
    llvm::Value* shifted_value = builder->CreateAShr(rt_value, builder->getInt32(total_shift));  // Arithmetic shift right (64-bit)

    // Store the result in the rd register
    builder->CreateStore(
        shifted_value,
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2))  // 64-bit offset for rd
    );

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_xori(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;   // Extract rt (bits 16-20)
    uint8_t rs = (opcode >> 21) & 0x1F;   // Extract rs (bits 21-25)
    uint16_t imm = static_cast<uint16_t>(opcode & 0xFFFF);  // Extract immediate (bits 0-15)

    // Load the base address of the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load the value from the rs register
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    // Load the immediate value (zero-extended to 32 bits)
    llvm::Value* imm_value = builder->getInt32(imm);

    // Perform XOR between GPR[rs] and the immediate
    llvm::Value* result = builder->CreateXor(rs_value, imm_value);

    // Store the result in GPR[rt]
    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4));
    builder->CreateStore(result, rt_ptr);

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_mult1(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)

    // Load values from registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Sign-extend rs_value and rt_value to 64 bits (to ensure multiplication works properly with signed values)
    llvm::Value* rs_value_64 = builder->CreateSExt(rs_value, builder->getInt64Ty());
    llvm::Value* rt_value_64 = builder->CreateSExt(rt_value, builder->getInt64Ty());

    // Perform 64-bit signed multiplication (product is 64 bits)
    llvm::Value* product = builder->CreateMul(rs_value_64, rt_value_64);

    // Extract the low 32 bits of the product for LO1 register (LO127..64)
    llvm::Value* lo1_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->lo) + 8), // Offset by 8 bytes for LO1
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* lo1_value = builder->CreateTrunc(product, builder->getInt32Ty()); // Lower 32 bits of product
    builder->CreateStore(lo1_value, lo1_ptr);

    // Extract the high 32 bits of the product for HI1 register (HI127..64)
    llvm::Value* hi1_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->hi) + 8), // Offset by 8 bytes for HI1
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* hi1_value = builder->CreateTrunc(builder->CreateLShr(product, builder->getInt64(32)), builder->getInt32Ty()); // Upper 32 bits of product
    builder->CreateStore(hi1_value, hi1_ptr);

    // Store the lower 32 bits of the product in GPR[rd] (if rd is not zero)
    llvm::Value* zero = builder->getInt32(0);
    llvm::Value* rd_not_zero = builder->CreateICmpNE(builder->getInt32(rd), zero);

    llvm::Value* rd_ptr = builder->CreateGEP(
        builder->getInt32Ty(),
        gpr_base,
        builder->getInt32(rd * 4)
    );
    llvm::Value* select_value = builder->CreateSelect(rd_not_zero, lo1_value, builder->getInt32(0)); // If rd is not zero, store lo1_value
    builder->CreateStore(select_value, rd_ptr);

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_movz(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract the registers from the opcode
    uint32_t rs = (opcode >> 21) & 0x1F;  // Bits 21-25
    uint32_t rt = (opcode >> 16) & 0x1F;  // Bits 16-20
    uint32_t rd = (opcode >> 11) & 0x1F;  // Bits 11-15

    // Load the base address of the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load the values from the registers rs and rt
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Check if rt is zero (MOVZ condition)
    llvm::Value* condition = builder->CreateICmpEQ(rt_value, builder->getInt32(0));

    // If condition is true (rt == 0), store the value of rs into rd
    llvm::BasicBlock* store_block = llvm::BasicBlock::Create(*context, "store_block", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* after_store_block = llvm::BasicBlock::Create(*context, "after_store_block", builder->GetInsertBlock()->getParent());

    // Conditional branch based on rt == 0
    builder->CreateCondBr(condition, store_block, after_store_block);

    // Store operation block (only executes if rt == 0)
    builder->SetInsertPoint(store_block);
    builder->CreateStore(rs_value, builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rd * 4)));

    // Continue after the store block
    builder->CreateBr(after_store_block);

    // After store block (if condition was not met, control will jump here)
    builder->SetInsertPoint(after_store_block);

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_dsllv(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract the registers from the opcode
    uint32_t rs = (opcode >> 21) & 0x1F;  // Bits 21-25
    uint32_t rt = (opcode >> 16) & 0x1F;  // Bits 16-20
    uint32_t rd = (opcode >> 11) & 0x1F;  // Bits 11-15

    // Load the base address of the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty()) // 64-bit signed integers
    );

    // Load the value from the rt register
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2)) // Access rt register
    );

    // Load the value from the rs register (shift amount)
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rs * 2)) // Access rs register
    );

    // Extract the low-order 6 bits from GPR[rs]
    llvm::Value* shift_amount = builder->CreateAnd(
        rs_value,
        llvm::ConstantInt::get(builder->getInt64Ty(), 0x3F) // Mask to lower 6 bits
    );

    // Perform the logical left shift (GPR[rd] = GPR[rt] << shift_amount)
    llvm::Value* shifted_value = builder->CreateShl(rt_value, shift_amount);

    // Store the result in the rd register
    builder->CreateStore(
        shifted_value,
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2)) // rd * 2 for 64-bit offset
    );

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_daddiu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract rt, rs, and immediate fields from the opcode
    uint8_t rt = (opcode >> 16) & 0x1F;  // Bits 16-20
    uint8_t rs = (opcode >> 21) & 0x1F;  // Bits 21-25
    int16_t imm = static_cast<int16_t>(opcode & 0xFFFF);  // Sign-extend the 16-bit immediate value

    // Get the base address of the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty()) // Registers are 64-bit
    );

    // Load the value from GPR[rs]
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rs * 2)) // Access rs register (64-bit offset)
    );

    // Sign-extend the immediate value to 64 bits
    llvm::Value* imm_value = builder->getInt64(imm);

    // Perform the addition: GPR[rt] = GPR[rs] + sign_extend(immediate)
    llvm::Value* result = builder->CreateAdd(rs_value, imm_value);

    // Store the result in GPR[rt]
    builder->CreateStore(
        result,
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2)) // Access rt register (64-bit offset)
    );

    // Update the program counter after executing the instruction
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_sq(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)

    // Get the GPR base pointer
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt128Ty()) // Access 128-bit registers directly
    );

    // Load the value of the base register (rs)
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt64Ty(), // rs is a 64-bit value (used for addressing)
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt32(rs * 2))
    );

    // Compute the effective address (addr = base + offset)
    llvm::Value* offset_value = builder->getInt64(offset); // Offset is extended to 64-bit
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);

    // Get the pointer to the 128-bit value of the rt register
    llvm::Value* rt_value_ptr = builder->CreateGEP(
        builder->getInt128Ty(), gpr_base, builder->getInt32(rt)
    );

    // Load the 128-bit value from the rt register
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt128Ty(), rt_value_ptr);

    // Split the 128-bit value into lower and upper 64 bits
    llvm::Value* upper_value_to_store = builder->CreateTrunc(rt_value, builder->getInt64Ty());
    llvm::Value* lower_value_to_store = builder->CreateTrunc(
        builder->CreateLShr(rt_value, builder->getIntN(128, 64)), // Logical shift right by 64 bits
        builder->getInt64Ty()
    );

    // Call ee_write64 for the lower 64 bits
    builder->CreateCall(
        ee_write64,
        {
            llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
            addr_value,
            lower_value_to_store
        }
    );

    // Increment the address by 8 for the upper 64 bits
    llvm::Value* addr_upper = builder->CreateAdd(
        addr_value,
        builder->getInt64(8) // Offset by 8 bytes for the upper 64 bits
    );

    // Call ee_write64 for the upper 64 bits
    builder->CreateCall(
        ee_write64,
        {
            llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
            addr_upper,
            upper_value_to_store
        }
    );

    // Update the program counter
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_lq(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);  // Extract the offset (16-bit immediate)

    // Get the GPR base pointer (points to the array of 128-bit registers)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt128Ty()) // Access 128-bit registers directly
    );

    // Load the value of the base register (rs)
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt64Ty(), // rs is a 64-bit value (used for addressing)
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt32(rs * 2))
    );

    // Extend the offset to 64-bit (signed extension)
    llvm::Value* offset_value = builder->getInt64(offset);

    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);

    // Call ee_read64 for the lower 64 bits
    llvm::Value* lower_value = builder->CreateCall(ee_read64, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
        addr_value
    });

    // Calculate the address for the upper 64 bits: addr_upper = addr + 8
    llvm::Value* addr_upper = builder->CreateAdd(
        addr_value,
        builder->getInt64(8) // Offset by 8 bytes for the upper 64 bits
    );

    // Call ee_read64 for the upper 64 bits
    llvm::Value* upper_value = builder->CreateCall(ee_read64, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
        addr_upper
    });

    // Store the 128-bit value into the destination register (rt)
    builder->CreateStore(
        lower_value,
        builder->CreateGEP(
            builder->getInt64Ty(), gpr_base, builder->getInt64(rt) // GPR uses 128-bit registers
        )
    );

    // Store the 128-bit value into the destination register (rt)
    builder->CreateStore(
        upper_value,
        builder->CreateGEP(
            builder->getInt64Ty(), gpr_base, builder->getInt64(rt + 1) // GPR uses 128-bit registers
        )
    );

    // Update the program counter
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_dsrl(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract the destination register (rd)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract the source register (rt)
    uint8_t sa = (opcode >> 6) & 0x1F;   // Extract the shift amount (sa)

    // Handle 128-bit register access for rt (source register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // Each register is 8-bit
    );

    // Load the 64-bit value from the rt register
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt64Ty(), builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2) // GPR uses 64-bit (8 bytes)
    ));

    // Create the shift amount (sa), which is already a 5-bit number
    llvm::Value* shift_amount = builder->getInt32(sa); // sa is between 0 and 31, cast to 32-bit

    // Perform the logical right shift on rt by sa
    llvm::Value* result = builder->CreateLShr(rt_value, shift_amount); // Logical Shift Right (LShr)

    // Store the result into the rd register
    builder->CreateStore(result, builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt32(rd * 2) // GPR uses 64-bit registers (8 bytes)
    ));

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_lh(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t base = (opcode >> 21) & 0x1F;  // Extract BASE (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;    // Extract RT (bits 16-20)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);  // Extract OFFSET (bits 0-15)

    // Load the base register value
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())  // Pointer to a 32-bit integer
    );
    llvm::Value* base_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(base * 4))
    );

    // Calculate the effective address
    llvm::Value* offset_value = builder->getInt32(offset);
    llvm::Value* addr_value = builder->CreateAdd(base_value, offset_value);

    // Call the `ee_read16` function to load the 16-bit value
    llvm::Value* value_to_load = builder->CreateCall(ee_read16, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
        addr_value
    });

    // Sign-extend the loaded 16-bit value to 32 bits
    llvm::Value* sign_extended_value = builder->CreateSExt(value_to_load, builder->getInt32Ty());

    // Store the result in the target register
    llvm::Value* gpr_rt = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    builder->CreateStore(
        sign_extended_value,
        builder->CreateGEP(builder->getInt32Ty(), gpr_rt, builder->getInt32(rt * 4))
    );

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}


void EEJIT::ee_jit_cache(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // TODO?

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_sllv(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Decode opcode fields
    uint8_t rd = (opcode >> 11) & 0x1F;  // Destination register (rd)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Source register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Shift amount register (rs)

    // Get the base pointer to the GPR registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // GPR base pointer (byte-addressable)
    );

    // Load the value from the rt register (value to shift)
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4) // GPR uses 32-bit words for UL[4]
    ));

    // Load the value from the rs register (shift amount)
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4) // GPR uses 32-bit words for UL[4]
    ));

    // Extract the lower 5 bits of rs_value for the shift amount
    llvm::Value* shift_amount = builder->CreateAnd(rs_value, builder->getInt32(0x1F));

    // Perform the logical left shift operation (SLLV)
    llvm::Value* result = builder->CreateShl(rt_value, shift_amount);

    result = builder->CreateSExt(result, builder->getInt64Ty());

    // Store the result into the rd register
    builder->CreateStore(result, builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2) // GPR uses 32-bit words for UL[4]
    ));

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_dsll(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract the registers and shift amount from the opcode
    uint32_t rt = (opcode >> 16) & 0x1F;  // Bits 16-20
    uint32_t rd = (opcode >> 11) & 0x1F;  // Bits 11-15
    uint32_t sa = (opcode >> 6) & 0x1F;   // Bits 6-10 (shift amount)
    
    // Load the base address of the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty()) // 64-bit integers (doubleword)
    );

    // Load the value from the rt register
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt64Ty(), // Load as 64-bit integer
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2)) // rt * 2 for 64-bit offset
    );

    // Perform the logical left shift (shift by sa)
    llvm::Value* shifted_value = builder->CreateShl(rt_value, builder->getInt32(sa));  // Logical shift left (64-bit shift)

    // Store the result in the rd register
    builder->CreateStore(
        shifted_value,
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2)) // rd * 2 for 64-bit offset
    );

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_srav(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract the registers from the opcode
    uint32_t rs = (opcode >> 21) & 0x1F;  // Bits 21-25
    uint32_t rt = (opcode >> 16) & 0x1F;  // Bits 16-20
    uint32_t rd = (opcode >> 11) & 0x1F;  // Bits 11-15

    // Load the base address of the registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty()) // 64-bit signed integers
    );

    // Load the values from the registers rs and rt
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2)) // Access rt register
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rs * 2)) // Access rs register
    );

    // Mask the shift amount to ensure it's within the range 0-31 (lower 5 bits)
    llvm::Value* shift_amount = builder->CreateAnd(rs_value, llvm::ConstantInt::get(builder->getInt64Ty(), 0x1F)); // Mask to lower 5 bits

    // Perform the arithmetic right shift (signed shift)
    llvm::Value* shifted_value = builder->CreateAShr(rt_value, shift_amount); // Arithmetic right shift

    // Sign-extend the result to 64 bits
    llvm::Value* sign_extended_value = builder->CreateSExt(shifted_value, builder->getInt64Ty());

    // Store the result in the rd register
    builder->CreateStore(sign_extended_value, builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2)));

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_nor(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Extract registers from opcode
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)

    // Create pointers to the general-purpose registers (GPR)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load the value from the RS register
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    // Load the value from the RT register
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Perform the bitwise OR operation
    llvm::Value* or_result = builder->CreateOr(rs_value, rt_value);

    // Perform the bitwise NOT operation
    llvm::Value* nor_result = builder->CreateNot(or_result);

    // Store the result in the RD register
    llvm::Value* rd_ptr = builder->CreateGEP(
        builder->getInt32Ty(),
        gpr_base,
        builder->getInt32(rd * 4)
    );
    builder->CreateStore(nor_result, rd_ptr);

    // Emit the program counter update
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_cfc2(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;
    uint8_t rt = (opcode >> 16) & 0x1F;

    // Pointer to the VU0 control register
    llvm::Value* vu0_reg_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->vu0.vi)),
        llvm::PointerType::getUnqual(builder->getInt16Ty())
    );

    // Load the value from the VU0 control register
    llvm::Value* vu0_value = builder->CreateLoad(
        builder->getInt16Ty(),
        builder->CreateGEP(builder->getInt16Ty(), vu0_reg_base, builder->getInt32(rd))
    );

    // Extend the value to 64-bit
    llvm::Value* vu0_val_ze = builder->CreateZExt(vu0_value, builder->getInt64Ty());

    // Store the value to the general-purpose register
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* gpr_u32_0 = builder->CreateGEP(builder->getInt64Ty(), gpr_base, {builder->getInt64(rt * 2)});
    builder->CreateStore(vu0_val_ze, gpr_u32_0);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_ctc2(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;
    uint8_t rt = (opcode >> 16) & 0x1F;

    // Create pointers to the general-purpose registers (GPR)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load the value from the general-purpose register
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 2))
    );

    // Pointer to the VU0 control register
    llvm::Value* vu0_reg_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->vu0.vi)),
        llvm::PointerType::getUnqual(builder->getInt16Ty())
    );

    // Store the value to the VU0 control register
    llvm::Value* vu0_reg_ptr = builder->CreateGEP(
        builder->getInt16Ty(),
        vu0_reg_base,
        builder->getInt32(rd)
    );
    builder->CreateStore(rt_value, vu0_reg_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_lwu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)

    // Handle 128-bit register access for rs (base register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // Each register is 8-bit
    );

    // Load the base register value (rs) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4) // GPR uses 32-bit (4 bytes) offset
    ));

    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);

    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);

    // Call read32: read32(core, addr)
    llvm::Value* value_to_load = builder->CreateCall(ee_read32, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
        addr_value
    });

    // Zero-extend the loaded value to 64 bits
    llvm::Value* zero_extended_value = builder->CreateZExt(value_to_load, builder->getInt64Ty());

    // Store the zero-extended value into register rt
    builder->CreateStore(zero_extended_value, builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt32(rt * 2)
    ));

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_ldl(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)

    // Handle 128-bit register access for rs (base register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // Each register is 8-bit
    );

    // Load the base register value (rs) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4) // GPR uses 32-bit (4 bytes) offset
    ));

    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);

    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);

    // Determine the byte offset within the doubleword
    llvm::Value* byte_offset = builder->CreateAnd(addr_value, builder->getInt32(0x7));

    // Align the address to the nearest doubleword boundary
    llvm::Value* aligned_addr = builder->CreateAnd(addr_value, builder->getInt32(~0x7));

    // Call read64 to fetch the doubleword from memory
    llvm::Value* value_to_load = builder->CreateCall(ee_read64, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
        aligned_addr
    });

    // Adjust the value based on the byte offset
    llvm::Value* shift_amount = builder->CreateMul(byte_offset, builder->getInt32(8));
    llvm::Value* loaded_value = builder->CreateLShr(value_to_load, shift_amount);

    // Create a mask for merging the loaded value with the existing register value
    llvm::Value* mask = builder->CreateShl(builder->getInt64(0xFFFFFFFFFFFFFFFF), shift_amount);
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt64Ty(), builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt32(rt * 2)
    ));
    llvm::Value* merged_value = builder->CreateOr(builder->CreateAnd(rt_value, builder->CreateNot(mask)), builder->CreateAnd(loaded_value, mask));

    // Store the merged value into register rt
    builder->CreateStore(merged_value, builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt32(rt * 2)
    ));

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_ldr(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)

    // Handle 128-bit register access for rs (base register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // Each register is 8-bit
    );

    // Load the base register value (rs) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4) // GPR uses 32-bit (4 bytes) offset
    ));

    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);

    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);

    // Determine the byte offset within the doubleword
    llvm::Value* byte_offset = builder->CreateAnd(addr_value, builder->getInt32(0x7));

    // Align the address to the nearest doubleword boundary
    llvm::Value* aligned_addr = builder->CreateAnd(addr_value, builder->getInt32(~0x7));

    // Call read64 to fetch the doubleword from memory
    llvm::Value* value_to_load = builder->CreateCall(ee_read64, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
        aligned_addr
    });

    // Adjust the value based on the byte offset
    llvm::Value* shift_amount = builder->CreateMul(builder->CreateSub(builder->getInt32(7), byte_offset), builder->getInt32(8));
    llvm::Value* loaded_value = builder->CreateShl(value_to_load, shift_amount);

    // Create a mask for merging the loaded value with the existing register value
    llvm::Value* mask = builder->CreateLShr(builder->getInt64(0xFFFFFFFFFFFFFFFF), shift_amount);
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt64Ty(), builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt32(rt * 2)
    ));
    llvm::Value* merged_value = builder->CreateOr(builder->CreateAnd(rt_value, builder->CreateNot(mask)), builder->CreateAnd(loaded_value, mask));

    // Store the merged value into register rt
    builder->CreateStore(merged_value, builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt32(rt * 2)
    ));

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_sdl(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the source register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)

    // Handle 128-bit register access for rs (base register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // Each register is 8-bit
    );

    // Load the base register value (rs) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4) // GPR uses 32-bit (4 bytes) offset
    ));

    // Load the source register value (rt) from the GPR array
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt64Ty(), builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt32(rt * 2)
    ));

    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);

    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);

    // Determine the byte offset within the doubleword
    llvm::Value* byte_offset = builder->CreateAnd(addr_value, builder->getInt32(0x7));

    // Align the address to the nearest doubleword boundary
    llvm::Value* aligned_addr = builder->CreateAnd(addr_value, builder->getInt32(~0x7));

    // Adjust the value based on the byte offset
    llvm::Value* shift_amount = builder->CreateMul(byte_offset, builder->getInt32(8));
    llvm::Value* data_to_store = builder->CreateShl(rt_value, shift_amount);

    // Call write64 to store the doubleword in memory
    builder->CreateCall(ee_write64, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
        aligned_addr,
        data_to_store
    });

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_sdr(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the source register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)

    // Handle 128-bit register access for rs (base register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // Each register is 8-bit
    );

    // Load the base register value (rs) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4) // GPR uses 32-bit (4 bytes) offset
    ));

    // Load the source register value (rt) from the GPR array
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt64Ty(), builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt32(rt * 2)
    ));

    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);

    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);

    // Determine the byte offset within the doubleword
    llvm::Value* byte_offset = builder->CreateAnd(addr_value, builder->getInt32(0x7));

    // Align the address to the nearest doubleword boundary
    llvm::Value* aligned_addr = builder->CreateAnd(addr_value, builder->getInt32(~0x7));

    // Adjust the value based on the byte offset
    llvm::Value* shift_amount = builder->CreateMul(builder->CreateSub(builder->getInt32(7), byte_offset), builder->getInt32(8));
    llvm::Value* data_to_store = builder->CreateLShr(rt_value, shift_amount);

    // Call write64 to store the doubleword in memory
    builder->CreateCall(ee_write64, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
        aligned_addr,
        data_to_store
    });

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_srlv(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract the destination register (rd)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract the source register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract the shift amount register (rs)

    // Handle 128-bit register access for rs (shift amount register) and rt (source register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // Each register is 8-bit
    );

    // Load the 32-bit value from the rs register (shift amount)
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4) // GPR uses 32-bit (4 bytes)
    ));

    // Load the 32-bit value from the rt register (source value)
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4) // GPR uses 32-bit (4 bytes)
    ));

    // Extract the lowest 5 bits of the rs value to get the shift amount
    llvm::Value* shift_amount = builder->CreateAnd(rs_value, builder->getInt32(0x1F)); // 5-bit mask

    // Perform the logical right shift on rt by the shift amount in rs
    llvm::Value* result = builder->CreateLShr(rt_value, shift_amount); // Logical Shift Right (LShr)

    // Store the result into the rd register
    builder->CreateStore(result, builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rd * 4) // GPR uses 32-bit registers (4 bytes)
    ));

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_dsrl32(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract the destination register (rd)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract the source register (rt)
    uint8_t sa = (opcode >> 6) & 0x1F;   // Extract the shift amount (sa)

    // Handle 128-bit register access for rt (source register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // Each register is 8-bit
    );

    // Load the 64-bit value from the rt register
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt64Ty(), builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2) // GPR uses 64-bit (8 bytes)
    ));

    // Compute the total shift amount: sa + 32
    llvm::Value* total_shift = builder->getInt32(32 + sa);

    // Perform the logical right shift on rt by total_shift
    llvm::Value* result = builder->CreateLShr(rt_value, total_shift); // Logical Shift Right (LShr)

    // Store the result into the rd register
    builder->CreateStore(result, builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt32(rd * 2) // GPR uses 64-bit registers (8 bytes)
    ));

    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_cop1(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_cop2(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_padduw(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Decode opcode fields
    uint8_t rs = (opcode >> 21) & 0x1F;  // Source register (rs)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Source register (rt)
    uint8_t rd = (opcode >> 11) & 0x1F;  // Destination register (rd)

    // Base pointer to the GPR registers
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );

    // Helper function to get a pointer to the GPR word
    auto get_gpr_word_ptr = [&](uint8_t reg, uint8_t word_idx) -> llvm::Value* {
        return builder->CreateGEP(
            builder->getInt32Ty(),
            builder->CreateBitCast(gpr_base, llvm::PointerType::get(builder->getInt32Ty(), 0)),
            builder->getInt32(reg * 4 + word_idx)  // Each GPR is 128 bits (4 x 32-bit words)
        );
    };

    // Helper to load a 32-bit word from GPR
    auto load_gpr_word = [&](uint8_t reg, uint8_t word_idx) -> llvm::Value* {
        return builder->CreateLoad(builder->getInt32Ty(), get_gpr_word_ptr(reg, word_idx));
    };

    // Helper to store a 32-bit word into a GPR
    auto store_gpr_word = [&](uint8_t reg, uint8_t word_idx, llvm::Value* value) {
        builder->CreateStore(value, get_gpr_word_ptr(reg, word_idx));
    };

    // Iterate over each of the four 32-bit words
    for (uint8_t word_idx = 0; word_idx < 4; ++word_idx) {
        // Load the values from the source registers
        llvm::Value* rs_value = load_gpr_word(rs, word_idx);
        llvm::Value* rt_value = load_gpr_word(rt, word_idx);

        // Add the two values as 64-bit integers to handle overflow
        llvm::Value* extended_rs = builder->CreateZExt(rs_value, builder->getInt64Ty());
        llvm::Value* extended_rt = builder->CreateZExt(rt_value, builder->getInt64Ty());
        llvm::Value* sum = builder->CreateAdd(extended_rs, extended_rt);

        // Check for overflow (sum > 0xFFFFFFFF)
        llvm::Value* max_uint32 = builder->getInt64(0xFFFFFFFF);
        llvm::Value* is_overflow = builder->CreateICmpUGT(sum, max_uint32);

        // Select the saturated value (0xFFFFFFFF) or the sum truncated to 32 bits
        llvm::Value* saturated_value = builder->getInt32(0xFFFFFFFF);
        llvm::Value* result = builder->CreateSelect(
            is_overflow,
            saturated_value,
            builder->CreateTrunc(sum, builder->getInt32Ty())
        );

        // Store the result into the destination register
        store_gpr_word(rd, word_idx, result);
    }

    // Update the program counter after the instruction is executed
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_di(uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Get the pointer to the COP0 Status register
    llvm::Value* cop0_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->cop0)),
        llvm::PointerType::getUnqual(builder->getInt8Ty())
    );

    llvm::Value* status_ptr = builder->CreateGEP(
        builder->getInt32Ty(), // Access 32-bit Status field
        cop0_base,
        builder->getInt32(12 * 4) // Offset to Status (index 12 * 4 bytes)
    );

    // Load the current Status value
    llvm::Value* status = builder->CreateLoad(builder->getInt32Ty(), status_ptr);

    // Extract relevant bits
    llvm::Value* EDI = builder->CreateAnd(status, builder->getInt32(1 << 17)); // Bit 17
    llvm::Value* EXL = builder->CreateAnd(status, builder->getInt32(1 << 1));  // Bit 1
    llvm::Value* ERL = builder->CreateAnd(status, builder->getInt32(1 << 2));  // Bit 2
    llvm::Value* KSU = builder->CreateAnd(status, builder->getInt32(0b11 << 3)); // Bits 3-4

    // Check if (EDI || EXL || ERL || KSU == 0)
    llvm::Value* EDI_check = builder->CreateICmpNE(EDI, builder->getInt32(0));
    llvm::Value* EXL_check = builder->CreateICmpNE(EXL, builder->getInt32(0));
    llvm::Value* ERL_check = builder->CreateICmpNE(ERL, builder->getInt32(0));
    llvm::Value* KSU_check = builder->CreateICmpEQ(KSU, builder->getInt32(0)); // Kernel mode

    llvm::Value* condition = builder->CreateOr(
        builder->CreateOr(EDI_check, EXL_check),
        builder->CreateOr(ERL_check, KSU_check)
    );

    // If condition is true, clear EIE (bit 16)
    llvm::BasicBlock* then_block = llvm::BasicBlock::Create(*context, "then", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* else_block = llvm::BasicBlock::Create(*context, "else", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* merge_block = llvm::BasicBlock::Create(*context, "merge", builder->GetInsertBlock()->getParent());

    builder->CreateCondBr(condition, then_block, else_block);

    // THEN block: clear EIE
    builder->SetInsertPoint(then_block);
    llvm::Value* clear_EIE = builder->CreateAnd(status, builder->getInt32(~(1 << 16)));
    builder->CreateStore(clear_EIE, status_ptr);
    builder->CreateBr(merge_block);

    // ELSE block: do nothing
    builder->SetInsertPoint(else_block);
    builder->CreateBr(merge_block);

    // Merge block: continue execution
    builder->SetInsertPoint(merge_block);

    // Emit the PC update
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_eret(uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    auto& cop0 = core->cop0;

    is_branch = true;

    // Prepare LLVM values for COP0 registers
    llvm::Value* status_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&cop0.Status)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* error_epc_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&cop0.errorEPC)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* epc_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&cop0.epc)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load current values
    llvm::Value* status = builder->CreateLoad(builder->getInt32Ty(), status_ptr);
    llvm::Value* error_epc = builder->CreateLoad(builder->getInt32Ty(), error_epc_ptr);
    llvm::Value* epc = builder->CreateLoad(builder->getInt32Ty(), epc_ptr);

    // Extract ERL and EXL bits
    llvm::Value* erl_bit = builder->CreateAnd(status, builder->getInt32(1 << 2)); // Bit 2: ERL
    llvm::Value* erl_check = builder->CreateICmpNE(erl_bit, builder->getInt32(0));

    // Create blocks for branching
    llvm::Function* parent_function = builder->GetInsertBlock()->getParent();
    llvm::BasicBlock* erl_true_block = llvm::BasicBlock::Create(*context, "erl_1", parent_function);
    llvm::BasicBlock* erl_false_block = llvm::BasicBlock::Create(*context, "erl_0", parent_function);
    llvm::BasicBlock* sideload_block = llvm::BasicBlock::Create(*context, "sideload", parent_function);
    llvm::BasicBlock* sideload_true_block = llvm::BasicBlock::Create(*context, "sideload_true", parent_function);
    llvm::BasicBlock* end_block = llvm::BasicBlock::Create(*context, "end", parent_function);

    // First branch: Check ERL
    builder->CreateCondBr(erl_check, erl_true_block, erl_false_block);

    // ERL block: Set PC to ErrorEPC and clear ERL
    builder->SetInsertPoint(erl_true_block);
    builder->CreateStore(error_epc, builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->pc)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    ));
    llvm::Value* clear_erl = builder->CreateAnd(status, builder->getInt32(~(1 << 2))); // Clear ERL (Bit 2)
    builder->CreateStore(clear_erl, status_ptr);
    builder->CreateBr(sideload_block);

    // Normal EPC block: Set PC to EPC and clear EXL
    builder->SetInsertPoint(erl_false_block);
    builder->CreateStore(epc, builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->pc)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    ));
    llvm::Value* clear_exl = builder->CreateAnd(status, builder->getInt32(~(1 << 1))); // Clear EXL (Bit 1)
    builder->CreateStore(clear_exl, status_ptr);
    builder->CreateBr(sideload_block);

    // Sideload block
    builder->SetInsertPoint(sideload_block);
    llvm::Value* sideload_elf_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&run_elf)),
        llvm::PointerType::getUnqual(builder->getInt1Ty())
    );
    llvm::Value* sideload_elf = builder->CreateLoad(builder->getInt1Ty(), sideload_elf_ptr);
    llvm::Value* sideload_check = builder->CreateICmpNE(sideload_elf, builder->getInt1(0));
    builder->CreateCondBr(sideload_check, sideload_true_block, end_block);

    // Sideload true block
    builder->SetInsertPoint(sideload_true_block);
    builder->CreateStore(builder->getInt1(0), sideload_elf_ptr);
    builder->CreateCall(ee_load_elf, {llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core))});

    llvm::Value* sideload_elf_entry_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->elf_entry_point)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* sideload_elf_entry = builder->CreateLoad(builder->getInt32Ty(), sideload_elf_entry_ptr);

    llvm::Value* pc_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->pc)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    builder->CreateStore(sideload_elf_entry, pc_ptr);

    builder->CreateBr(end_block);

    // End block: End the function
    builder->SetInsertPoint(end_block);
}

void EEJIT::ee_jit_syscall(uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    is_branch = true; // Mark as a branching instruction

    // Load GPR base pointer
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load rs and rt values
    llvm::Value* syscall_nr = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(3 * 2))
    );

    /*builder->CreateCall(ee_print_syscall, {
        llvm::ConstantInt::get(builder->getInt64Ty(),
        reinterpret_cast<uint64_t>(core)),
            builder->getInt32(current_pc),
            syscall_nr
    });*/

    ee_jit_level1_exception(core, CAUSE_SYSCALL, current_pc);
}

void EEJIT::ee_jit_bltzl(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract rs register
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract rt register
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);  // Extract and sign-extend offset

    is_branch = true;

    // Load GPR base pointer
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load rs and rt values
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );
    
    // Load rs and rt values
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // Create the condition: check if GPR[rs] is less than 0 (signed comparison)
    llvm::Value* condition = builder->CreateICmpSLT(rs_value, builder->getInt32(0));

    // Compute the branch destination: PC + (offset * 4)
    llvm::Value* branch_offset = builder->CreateSExt(builder->getInt32(offset), builder->getInt64Ty());
    llvm::Value* branch_dest = builder->CreateAdd(
        builder->getInt64(current_pc),
        builder->CreateMul(branch_offset, builder->getInt64(4))
    );

    // Update PC based on condition
    llvm::Value* pc_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->pc)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );

    llvm::Value* pc_new = builder->CreateSelect(
        condition,
        builder->CreateAdd(builder->getInt64(current_pc), builder->getInt64(4)),
        builder->CreateAdd(builder->getInt64(current_pc), builder->getInt64(8))
    );

    builder->CreateStore(pc_new, pc_ptr);

    // Update branching metadata
    llvm::Value* branching_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)),
        llvm::PointerType::getUnqual(builder->getInt1Ty())
    );

    llvm::Value* branching_new = builder->CreateSelect(
        condition,
        builder->getInt1(true),
        builder->getInt1(false)
    );

    builder->CreateStore(branching_new, branching_ptr);

    llvm::Value* branch_dest_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    builder->CreateStore(branch_dest, branch_dest_ptr);

    llvm::Value* likely_ptr = builder->CreateIntToPtr( \
                    builder->getInt64(reinterpret_cast<uint64_t>(&(core->likely_branch))),
                    llvm::PointerType::getUnqual(builder->getInt1Ty())
                );
    builder->CreateStore(builder->getTrue(), likely_ptr);
}

void EEJIT::ee_jit_sub(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)

    // Load values from registers GPR[rs] and GPR[rt]
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // TODO Overflow check exception?

    // Perform the subtraction without overflow check
    llvm::Value* result = builder->CreateSub(rs_value, rt_value);

    llvm::Value* res_se = builder->CreateSExt(result, builder->getInt64Ty());

    // Store the result into GPR[rd]
    llvm::Value* rd_ptr = builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2));
    builder->CreateStore(res_se, rd_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_add(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)

    // Load values from registers GPR[rs] and GPR[rt]
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );

    // TODO: Overflow check exception?
    // Perform the addition
    llvm::Value* result = builder->CreateAdd(rs_value, rt_value);

    llvm::Value* res_se = builder->CreateSExt(result, builder->getInt64Ty());

    // Store the result into GPR[rd]
    llvm::Value* rd_ptr = builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2));
    builder->CreateStore(res_se, rd_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_addi(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;
    uint8_t rs = (opcode >> 21) & 0x1F;
    int16_t imm = static_cast<int16_t>(opcode & 0xFFFF);  // Sign-extend immediate value

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4))
    );

    // TODO: Overflow check exception?

    // Sign-extend the immediate value to 64 bits
    llvm::Value* imm_value = builder->CreateSExt(builder->getInt32(imm), builder->getInt64Ty());
    llvm::Value* rs_value_64 = builder->CreateSExt(rs_value, builder->getInt64Ty());

    llvm::Value* result = builder->CreateAdd(rs_value_64, imm_value);

    llvm::Value* rt_ptr = builder->CreateGEP(
        builder->getInt32Ty(),
        gpr_base,
        builder->getInt32(rt * 4)
    );

    builder->CreateStore(result, rt_ptr);
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_ei(uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    // Get the pointer to the COP0 Status register.
    llvm::Value* cop0_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->cop0)),
        llvm::PointerType::getUnqual(builder->getInt8Ty())
    );

    llvm::Value* status_ptr = builder->CreateGEP(
        builder->getInt32Ty(), // We access the 32-bit Status field.
        cop0_base,
        builder->getInt32(12 * 4) // Offset to Status (index 12 * 4 bytes)
    );

    // Load the current Status value.
    llvm::Value* status = builder->CreateLoad(builder->getInt32Ty(), status_ptr);

    // Extract relevant bits from Status.
    llvm::Value* EDI = builder->CreateAnd(status, builder->getInt32(1 << 17)); // Bit 17
    llvm::Value* EXL = builder->CreateAnd(status, builder->getInt32(1 << 1));  // Bit 1
    llvm::Value* ERL = builder->CreateAnd(status, builder->getInt32(1 << 2));  // Bit 2
    llvm::Value* KSU = builder->CreateAnd(status, builder->getInt32(0b11 << 3)); // Bits 3-4

    // Create condition booleans.
    llvm::Value* EDI_check = builder->CreateICmpNE(EDI, builder->getInt32(0));
    llvm::Value* EXL_check = builder->CreateICmpNE(EXL, builder->getInt32(0));
    llvm::Value* ERL_check = builder->CreateICmpNE(ERL, builder->getInt32(0));
    llvm::Value* KSU_check = builder->CreateICmpEQ(KSU, builder->getInt32(0)); // Kernel mode check

    // Condition: if (EDI || EXL || ERL || (KSU == 0))
    llvm::Value* condition = builder->CreateOr(
        builder->CreateOr(EDI_check, EXL_check),
        builder->CreateOr(ERL_check, KSU_check)
    );

    // Create basic blocks for conditional branching.
    llvm::BasicBlock* then_block = llvm::BasicBlock::Create(*context, "then", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* else_block = llvm::BasicBlock::Create(*context, "else", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* merge_block = llvm::BasicBlock::Create(*context, "merge", builder->GetInsertBlock()->getParent());

    // Branch based on the condition.
    builder->CreateCondBr(condition, then_block, else_block);

    // THEN block: set EIE (bit 16) by OR'ing with (1 << 16)
    builder->SetInsertPoint(then_block);
    llvm::Value* set_EIE = builder->CreateOr(status, builder->getInt32(1 << 16));
    builder->CreateStore(set_EIE, status_ptr);
    builder->CreateBr(merge_block);

    // ELSE block: do nothing.
    builder->SetInsertPoint(else_block);
    builder->CreateBr(merge_block);

    // Merge block: continue execution.
    builder->SetInsertPoint(merge_block);

    // Emit the PC update.
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}
