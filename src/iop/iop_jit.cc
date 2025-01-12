#include <iop/iop_jit.hh>
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

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

IOPJIT::IOPJIT(IOP* core) : core(core) {
    // Initialize LLVM components
    llvm::InitializeNativeTarget();
    llvm::InitializeNativeTargetAsmPrinter();
    llvm::InitializeNativeTargetAsmParser();

    // Create context and builder
    context = std::make_unique<llvm::LLVMContext>();
    builder = std::make_unique<llvm::IRBuilder<>>(*context);

    // Create module
    module = std::make_unique<llvm::Module>("iop_jit", *context);
    
    // Create execution engine with module ownership
    std::string errStr;
    llvm::Module* modulePtr = module.get(); // Keep raw pointer
    
    executionEngine = std::unique_ptr<llvm::ExecutionEngine>(
        llvm::EngineBuilder(std::unique_ptr<llvm::Module>(modulePtr))
            .setErrorStr(&errStr)
            .setEngineKind(llvm::EngineKind::JIT)
            .create());

    if (!executionEngine) {
        Logger::error("Failed to create ExecutionEngine: " + errStr);
        return;
    }

    Logger::info("JIT initialization successful");
    initialize_opcode_table();

    iop_read32_type = llvm::FunctionType::get(builder->getInt32Ty(), {builder->getInt64Ty(), builder->getInt32Ty()}, false);
    iop_read32 = llvm::Function::Create(
    iop_read32_type, llvm::Function::ExternalLinkage, "iop_read32", module.get());;
    iop_write32_type = llvm::FunctionType::get(builder->getInt32Ty(), {builder->getInt64Ty(), builder->getInt32Ty(), builder->getInt32Ty()}, false);
    iop_write32 = llvm::Function::Create(
    iop_write32_type, llvm::Function::ExternalLinkage, "iop_write32", module.get());;

    ready = true;
}

IOPJIT::~IOPJIT() {
    if (builder) {
        builder.reset();
    }

    if (executionEngine) {
        if (module) {
            executionEngine->removeModule(module.get());
            module.reset();
        }
        executionEngine.reset();
    }

    if (context) {
        context.reset();
    }
}

void IOPJIT::initialize_opcode_table() {
    opcode_table[0x00].funct3_map[0x00] = &IOPJIT::iop_jit_sll; // SLL opcode
    opcode_table[0x00].funct3_map[0x08] = &IOPJIT::iop_jit_jr; // JR opcode
    opcode_table[0x00].funct3_map[0x25] = &IOPJIT::iop_jit_or; // OR opcode

    opcode_table[0x03].single_handler = &IOPJIT::iop_jit_jal; // JAL opcode
    opcode_table[0x04].single_handler = &IOPJIT::iop_jit_beq; // BEQ opcode
    opcode_table[0x05].single_handler = &IOPJIT::iop_jit_bne; // BNE opcode
    opcode_table[0x08].single_handler = &IOPJIT::iop_jit_addi; // ADDI opcode
    opcode_table[0x09].single_handler = &IOPJIT::iop_jit_addiu; // ADDIU opcode

    opcode_table[0x10].rs_map[0x00] = &IOPJIT::iop_jit_mfc0; // MFC0 opcode

    opcode_table[0x0A].single_handler = &IOPJIT::iop_jit_slti; // SLTI opcode
    opcode_table[0x0C].single_handler = &IOPJIT::iop_jit_andi; // ANDI opcode
    opcode_table[0x0D].single_handler = &IOPJIT::iop_jit_ori; // ORI opcode
    opcode_table[0x0F].single_handler = &IOPJIT::iop_jit_lui; // LUI opcode
    opcode_table[0x23].single_handler = &IOPJIT::iop_jit_lw; // LW opcode
    opcode_table[0x2B].single_handler = &IOPJIT::iop_jit_sw; // SW opcode
}

extern "C" uint32_t iop_read32(IOP* core, uint32_t addr) {
    return core->bus->read32(addr);
}

extern "C" void iop_write32(IOP* core, uint32_t addr, uint32_t value) {
    core->bus->write32(addr, value);
}

std::tuple<bool, uint32_t, bool> IOPJIT::generate_ir_for_opcode(uint32_t opcode, uint32_t current_pc) {
    bool is_branch = false;
    bool error = false;

    uint8_t opcode_index = (opcode >> 26) & 0x3F;
    uint8_t funct_or_rs = (opcode_index == 0x10) ? ((opcode >> 21) & 0x1F) : (opcode & 0x3F);

    auto it = opcode_table.find(opcode_index);
    if (it != opcode_table.end()) {
        OpcodeHandler handler = nullptr;

        if (opcode_index == 0x10) {
            auto rs_it = it->second.rs_map.find(funct_or_rs);
            if (rs_it != it->second.rs_map.end()) handler = rs_it->second;
        } else {
            auto funct_it = it->second.funct3_map.find(funct_or_rs);
            handler = (funct_it != it->second.funct3_map.end()) ? funct_it->second : it->second.single_handler;
        }

        if (handler) {
            (this->*(handler))(opcode, current_pc, is_branch, core);
        } else {
            base_error_handler(opcode);
        }
    } else {
        base_error_handler(opcode);
    }

    return {is_branch, current_pc, error};
}

void IOPJIT::base_error_handler(uint32_t opcode) {
    Logger::error("Unknown IOP LLVM IR opcode: " + format("0x{:08X}", opcode));
    Neo2::exit(1, Neo2::Subsystem::EE);
}

void IOPJIT::step() {
    single_instruction_mode = true;
    std::uint32_t opcode = core->fetch_opcode();
    execute_opcode(opcode);
    core->registers[0] = 0;
    single_instruction_mode = false;
}

void IOPJIT::run() {
    while (!Neo2::is_aborted()) {
        std::uint32_t opcode = core->fetch_opcode();
        execute_opcode(opcode);
        core->registers[0] = 0;
    }
}

void IOPJIT::execute_opcode(std::uint32_t opcode) {
    // Try to find existing block
    CompiledBlock* block = find_block(core->pc);
    
    if (!block) {
        // Compile new block if not found
        block = compile_block(core->pc, single_instruction_mode);
        if (!block) {
            return;
        }
        
        // Add to cache
        if (block_cache.size() >= CACHE_SIZE) {
            evict_oldest_block();
        }
        block_cache[core->pc] = *block;
    }
    
    // Execute block
    block->last_used = ++execution_count;
    auto exec_fn = (int (*)())block->code_ptr;
    int result = exec_fn();
}

CompiledBlock* IOPJIT::compile_block(uint32_t start_pc, bool single_instruction) {
    uint32_t current_pc = start_pc;
    uint32_t end_pc = start_pc; // Initialize end_pc to start_pc
    bool is_branch = false;
    
    // Create new module for this block
    auto new_module = std::make_unique<llvm::Module>(
        "block_" + std::to_string(start_pc), *context);
    
    if (!new_module) {
        Logger::error("Failed to create LLVM module");
        return nullptr;
    }
        
    // Create function and basic block
    llvm::FunctionType *funcType = llvm::FunctionType::get(builder->getInt32Ty(), false);
    llvm::Function *func = llvm::Function::Create(funcType, 
                                                llvm::Function::ExternalLinkage,
                                                "exec_" + std::to_string(start_pc),
                                                new_module.get());

    llvm::BasicBlock *entry = llvm::BasicBlock::Create(*context, "entry", func);
    builder->SetInsertPoint(entry);

    while (true) {
        uint32_t opcode = core->fetch_opcode(); // Fix function call
        
        // Generate IR for opcode
        auto [branch, current_pc_, error] = generate_ir_for_opcode(opcode, current_pc);

        is_branch = branch;

        if (error) {
            Logger::error("Error generating IR for opcode");
            Neo2::exit(1, Neo2::Subsystem::IOP);
            return nullptr;
        }

        // Check if opcode is a branch or jump
        if (is_branch || single_instruction) {
            end_pc = current_pc_;
            break;
        }

        current_pc = current_pc_;
    }

    builder->CreateRetVoid();

    std::string str;
    llvm::raw_string_ostream os(str);
    new_module->print(os, nullptr);
    os.flush();

    // Add module to execution engine
    if (!executionEngine) {
        Logger::error("Execution engine is not initialized");
        return nullptr;
    }

    executionEngine->addModule(std::move(new_module));
    executionEngine->finalizeObject();
    
    auto exec_fn = (void (*)())executionEngine->getPointerToFunction(func);
    if (!exec_fn) {
        Logger::error("Failed to JIT compile function");
        return nullptr;
    }

    // Create and return compiled block
    auto block = new CompiledBlock();
    block->start_pc = start_pc;
    block->end_pc = end_pc;
    block->code_ptr = (void*)exec_fn;
    block->last_used = execution_count;
    block->contains_branch = is_branch;
    block->llvm_ir = str;

    return block;
}

CompiledBlock* IOPJIT::find_block(uint32_t pc) {
    auto it = block_cache.find(pc);
    if (it != block_cache.end()) {
        return &it->second;
    }
    return nullptr;
}

void IOPJIT::evict_oldest_block() {
    if (lru_queue.empty()) return;
    uint32_t oldest_pc = lru_queue.front();
    lru_queue.erase(lru_queue.begin());
    block_cache.erase(oldest_pc);
}

void IOPJIT::iop_jit_mfc0(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;
    uint8_t rt = (opcode >> 16) & 0x1F;

    llvm::Value* cop0_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->cop0_registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* cop0_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(rd)));

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* gpr_u32 = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt));
    builder->CreateStore(cop0_value, gpr_u32);

    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}

void IOPJIT::iop_jit_addiu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;
    uint8_t rs = (opcode >> 21) & 0x1F;
    int16_t imm = static_cast<int16_t>(opcode & 0xFFFF); // Sign-extend immediate value

    llvm::Value* gpr_base = builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(core->registers)), llvm::PointerType::getUnqual(builder->getInt32Ty()));
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs)));
    llvm::Value* imm_value = builder->getInt32(imm);
    llvm::Value* result = builder->CreateAdd(rs_value, imm_value);
    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt));
    builder->CreateStore(result, rt_ptr);
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}


void IOPJIT::iop_jit_addi(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t imm = static_cast<int16_t>(opcode & 0xFFFF); // Extract the signed immediate (16-bit)
    // Get a pointer to the GPR base
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty()) // GPR is 32-bit
    );
    // Load the value from the rs register
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)
    ));
    // Create the immediate value (signed 16-bit immediate, sign-extended to 32-bit)
    llvm::Value* imm_value = builder->getInt32(imm);
    // Perform the addition: result = rs_value + imm_value
    llvm::Value* result = builder->CreateAdd(rs_value, imm_value);
    // Check for overflow (overflow occurs when the result can't fit in a signed 32-bit integer)
    llvm::Value* overflow_check = builder->CreateICmpSGT(result, builder->getInt32(INT32_MAX));
    llvm::BasicBlock* overflow_block = llvm::BasicBlock::Create(builder->getContext(), "overflow", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* no_overflow_block = llvm::BasicBlock::Create(builder->getContext(), "no_overflow", builder->GetInsertBlock()->getParent());
    // If overflow occurs, jump to the overflow block
    builder->CreateCondBr(overflow_check, overflow_block, no_overflow_block);
    // Overflow Block (handle overflow exception)
    builder->SetInsertPoint(overflow_block);
    // Implement exception handling here if necessary
    // For now, we can just raise an error (or can invoke a custom overflow handler)
    llvm::Value* overflow_msg = builder->CreateGlobalStringPtr("Overflow occurred in ADDI");
    
    // Get the module using the function's parent
    llvm::Module* module = builder->GetInsertBlock()->getParent()->getParent();
    
    // Call the trap intrinsic from the module
    builder->CreateCall(llvm::Intrinsic::getDeclaration(module, llvm::Intrinsic::trap), {});
    builder->CreateUnreachable();
    // No Overflow Block (continue with normal flow)
    builder->SetInsertPoint(no_overflow_block);
    // Store the result in the destination register (rt)
    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4));
    builder->CreateStore(result, rt_ptr);
    // Emit the update to PC after this operation
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}

void IOPJIT::iop_jit_sll(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;
    uint8_t rt = (opcode >> 16) & 0x1F;
    uint8_t sa = (opcode >> 6) & 0x1F;

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rt_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt)));
    llvm::Value* shifted_value = builder->CreateShl(rt_value, builder->getInt32(sa));
    llvm::Value* sign_extended_value = builder->CreateSExt(shifted_value, builder->getInt64Ty());

    llvm::Value* rd_ptr = builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt32(rd));
    builder->CreateStore(sign_extended_value, rd_ptr);

    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}

void IOPJIT::iop_jit_or(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract the destination register (rd)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract the source register (rs)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract the second source register (rt)
    // Get a pointer to the GPR base
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    // Load the values from the rs and rt registers
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs)
    ));
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rt)
    ));
    // Perform the OR operation: result = rs_value | rt_value
    llvm::Value* result = builder->CreateOr(rs_value, rt_value);
    // Store the result in the destination register (rd)
    llvm::Value* rd_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rd));
    builder->CreateStore(result, rd_ptr);
    // Emit the update to PC after this operation
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}
void IOPJIT::iop_jit_andi(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract the source register (rs)
    uint16_t imm = static_cast<uint16_t>(opcode & 0xFFFF);  // Extract the 16-bit immediate
    // Zero-extend the immediate to 32 bits
    uint32_t imm_value = imm;  // In C++, uint16_t will automatically zero-extend to uint32_t
    // Get a pointer to the GPR base
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    // Load the value from the rs register
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs)
    ));
    // Create the immediate value (zero-extended)
    llvm::Value* imm_value_llvm = builder->getInt32(imm_value);
    // Perform the AND operation: result = rs_value & imm_value
    llvm::Value* result = builder->CreateAnd(rs_value, imm_value_llvm);
    // Store the result in the destination register (rt)
    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt));
    builder->CreateStore(result, rt_ptr);
    // Emit the update to PC after this operation
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}

void IOPJIT::iop_jit_slti(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;
    uint8_t rs = (opcode >> 21) & 0x1F;
    int16_t imm = static_cast<int16_t>(opcode & 0xFFFF);

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs)));
    llvm::Value* imm_value = builder->CreateSExt(builder->getInt32(imm), builder->getInt32Ty());
    llvm::Value* result = builder->CreateICmpSLT(rs_value, imm_value);
    llvm::Value* result_int = builder->CreateZExt(result, builder->getInt32Ty());

    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt));
    builder->CreateStore(result_int, rt_ptr);

    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}


void IOPJIT::iop_jit_bne(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;
    uint8_t rt = (opcode >> 16) & 0x1F;
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs)));
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt)));
    llvm::Value* condition = builder->CreateICmpNE(rs_value, rt_value);

    llvm::BasicBlock* branch_block = llvm::BasicBlock::Create(*context, "branch", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* continue_block = llvm::BasicBlock::Create(*context, "continue", builder->GetInsertBlock()->getParent());

    builder->CreateCondBr(condition, branch_block, continue_block);

    // Branch block
    builder->SetInsertPoint(branch_block);
    llvm::Value* target_pc = builder->CreateAdd(builder->getInt32(current_pc), builder->getInt32(offset * 4));
    builder->CreateStore(target_pc, builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty())));
    builder->CreateStore(builder->getInt1(true), builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty())));
    builder->CreateBr(continue_block);

    // Continue block
    builder->SetInsertPoint(continue_block);
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);

    is_branch = true;
}

void IOPJIT::iop_jit_beq(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;
    uint8_t rt = (opcode >> 16) & 0x1F;
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);  // Get the 16-bit signed offset

    // Get the base address for the general-purpose registers (GPR)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load the values of rs and rt from the registers
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs)));
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt)));

    // Compare rs and rt for equality
    llvm::Value* condition = builder->CreateICmpEQ(rs_value, rt_value);

    // Create basic blocks for the branch (true) and continue (false)
    llvm::BasicBlock* branch_block = llvm::BasicBlock::Create(*context, "branch", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* continue_block = llvm::BasicBlock::Create(*context, "continue", builder->GetInsertBlock()->getParent());

    // Conditional branch based on the comparison
    builder->CreateCondBr(condition, branch_block, continue_block);

    // Branch block: if the condition is true, calculate the target PC and branch
    builder->SetInsertPoint(branch_block);

    // Sign-extend the offset to 32 bits
    llvm::Value* signed_offset = builder->CreateSExt(builder->getInt16(offset), builder->getInt32Ty()); // Sign-extend to 32-bit

    // Multiply by 4 (since MIPS is word-aligned)
    signed_offset = builder->CreateMul(signed_offset, builder->getInt32(4), "offset_mul_4");

    // Add 4 to current_pc to get to the next instruction
    llvm::Value* current_pc_value = builder->getInt32(static_cast<int32_t>(current_pc)); // Convert current_pc to signed 32-bit integer
    llvm::Value* next_pc_value = builder->CreateAdd(current_pc_value, builder->getInt32(4), "next_pc");  // current_pc + 4 (next instruction)

    // Add the signed offset to get the target PC
    llvm::Value* target_pc = builder->CreateAdd(next_pc_value, signed_offset, "calc_target_pc");

    // Store the target PC and mark the branch destination
    builder->CreateStore(target_pc, builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    ));

    // Set the branching flag to true
    builder->CreateStore(builder->getInt1(true), builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)),
        llvm::PointerType::getUnqual(builder->getInt1Ty())
    ));

    // Branch to the continue block after handling the branch
    builder->CreateBr(continue_block);

    // Continue block: update the PC if no branch was taken
    builder->SetInsertPoint(continue_block);
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);

    // Set the flag indicating a branch was taken
    is_branch = true;
}

void IOPJIT::iop_jit_lui(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;
    int16_t imm = static_cast<int16_t>(opcode & 0xFFFF);

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* imm_value = builder->CreateShl(builder->getInt32(imm), builder->getInt32(16));
    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt));
    builder->CreateStore(imm_value, rt_ptr);

    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}

void IOPJIT::iop_jit_ori(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;
    uint8_t rs = (opcode >> 21) & 0x1F;
    uint16_t imm = static_cast<uint16_t>(opcode & 0xFFFF);

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs)));
    llvm::Value* imm_value = builder->getInt32(imm);
    llvm::Value* result = builder->CreateOr(rs_value, imm_value);

    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt));
    builder->CreateStore(result, rt_ptr);

    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}

void IOPJIT::iop_jit_jr(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs)));

    builder->CreateStore(rs_value, builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty())));
    builder->CreateStore(builder->getInt1(true), builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty())));

    EMIT_IOP_UPDATE_PC(core, builder, current_pc);

    is_branch = true;
}

void IOPJIT::iop_jit_jal(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint32_t target = opcode & 0x03FFFFFF; // Extract the 26-bit target address
    // Shift the target address left by 2 (since instructions are 4 bytes)
    uint32_t target_address = (target << 2);
    // Calculate the return address (next instruction's address)
    uint32_t return_address = current_pc + 4;
    // Store the return address in register $31 (the link register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    
    llvm::Value* ra_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(31)); // $31 = $ra
    builder->CreateStore(builder->getInt32(return_address), ra_ptr);
    // Set the branch destination to the calculated target address
    builder->CreateStore(builder->getInt32(target_address), builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    ));
    // Mark the instruction as a branch
    builder->CreateStore(builder->getInt1(true), builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)),
        llvm::PointerType::getUnqual(builder->getInt1Ty())
    ));
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
    // Set the branch flag
    is_branch = true;
}

void IOPJIT::iop_jit_lw(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)
    uint32_t addr = core->registers[rs] + offset;
    llvm::Value* core_value = llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core));
    // Create the arguments for read32
    llvm::Value* addr_value = llvm::ConstantInt::get(builder->getInt32Ty(), addr);
    // Call read32: read32(core, addr)
    llvm::Value* value = builder->CreateCall(iop_read32, {core_value, addr_value});
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* gpr_u32_0 = builder->CreateGEP(builder->getInt32Ty(), gpr_base, {builder->getInt32(rt)});
    builder->CreateStore(value, gpr_u32_0);
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}

void IOPJIT::iop_jit_sw(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
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
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs)
    ));
    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);
    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value); 
    // Load the value to store from register rt
    llvm::Value* value_to_store = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(
        builder->getInt32Ty(), gpr_base, builder->getInt32(rt)
    ));
    // Call write32: write32(core, addr, value_to_store)
    builder->CreateCall(iop_write32, {llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
                                    addr_value, value_to_store});
    // Emit the update to PC after this operation
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}
