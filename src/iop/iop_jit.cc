#include <iop/iop_jit.hh>
#include <log/log.hh>
#include <neo2.hh>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/ExecutionEngine/Orc/LLJIT.h>
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

    // Create LLJIT instance
    auto jit = llvm::orc::LLJITBuilder().create();
    if (!jit) {
        Logger::error("Failed to create LLJIT: " + llvm::toString(jit.takeError()));
        printf("Failed to create LLJIT: %s\n", llvm::toString(jit.takeError()).c_str());
        fflush(stdout);
        return;
    }
    lljit = std::move(*jit);

    Logger::info("JIT initialization successful");
    initialize_opcode_table();

    ready = true;
}

IOPJIT::~IOPJIT() {
    if (builder) {
        builder.reset();
    }

    if (context) {
        context.reset();
    }
}


uint32_t IOPJIT::execute_block(Breakpoint *breakpoints)
{
    // Try to find existing block
    CompiledBlock *block = find_block(core->pc);

    if (!block)
    {
        // Compile a new block if not found
        block = compile_block(core->pc, breakpoints);
        if (!block)
        {
            Logger::error("Error compiling block!");
            return Neo2::exit(1, Neo2::Subsystem::IOP);
        }

        // Add to cache
        if (block_cache.size() >= CACHE_SIZE)
        {
            evict_oldest_block();
        }
        block_cache[core->pc] = *block;
    }

    // Execute block
    block->last_used = ++execution_count;
    auto exec_fn = (int (*)())block->code_ptr;
    core->cycles += block->cycles;
    exec_fn();
    return block->cycles;
}

CompiledBlock *IOPJIT::compile_block(uint32_t start_pc, Breakpoint *breakpoints)
{
    uint32_t current_pc = start_pc;
    uint32_t end_pc = start_pc; // Initialize end_pc to start_pc
    bool is_branch = false;

    // Create a new module for this block
    auto new_module = std::make_unique<llvm::Module>("block_0x" + format("{:08X}", start_pc), *context);

    if (!new_module)
    {
        Logger::error("Failed to create LLVM module");
        return nullptr;
    }

    // Create function and basic block
    llvm::FunctionType *funcType = llvm::FunctionType::get(builder->getInt32Ty(), false);
    llvm::Function *func = llvm::Function::Create(funcType, llvm::Function::ExternalLinkage,
                                                  "exec_0x" + format("{:08X}", start_pc), new_module.get());

    llvm::BasicBlock *entry = llvm::BasicBlock::Create(*context, "entry", func);
    builder->SetInsertPoint(entry);

    uint64_t old_cycles = core->cycles;
    uint64_t block_cycles_ = 0;

    setup_iop_jit_primitives(new_module); // Pass new_module to setup_ìop_jit_primitives

    while (!Neo2::is_aborted())
    {
        if (breakpoints && breakpoints->has_breakpoint(current_pc, CoreType::IOP))
        {
            // Notify the main thread
            Neo2::pause_emulation();

            // Set breakpoint information in the debug interface
            breakpoints->notify_breakpoint(current_pc);

            goto compile_exit;
        }

        if (core->branching)
        {
            if (single_instruction_mode)
                goto cont;

            if (!single_instruction_mode)
                core->branching = false; // Reset branching state
            uint32_t delay_slot_pc = current_pc;
            // Process the branch delay slot instruction
            uint32_t opcode = core->fetch_opcode(delay_slot_pc);
            auto [branch, _, error] = generate_ir_for_opcode(opcode, delay_slot_pc);
            if (error)
            {
                Logger::error("Error processing likely delay slot opcode");
                Neo2::exit(1, Neo2::Subsystem::IOP);
                return nullptr;
            }

            current_pc = core->branch_dest + 4; // Set PC to the branch destination
        }

    cont:
        uint32_t opcode = core->fetch_opcode(current_pc); // Fetch the next opcode

        // Generate IR for the opcode
        auto [branch, current_pc_, error] = generate_ir_for_opcode(opcode, current_pc);

        is_branch = branch;

        if (error)
        {
            Logger::error("Error generating IR for opcode");
            Neo2::exit(1, Neo2::Subsystem::IOP);
            return nullptr;
        }

        // Check if the opcode is a branch or jump
        if (is_branch || single_instruction_mode)
        {
            if (!single_instruction_mode)
            {
                current_pc += 4;
                opcode = core->fetch_opcode(current_pc);

                auto [branch, current_pc_, error] = generate_ir_for_opcode(opcode, current_pc);

                if (error)
                {
                    Logger::error("Error generating IR for branch-delay opcode");
                    Neo2::exit(1, Neo2::Subsystem::IOP);
                    return nullptr;
                }
            }

            end_pc = current_pc_;
            break;
        }

        current_pc = current_pc_; // Update PC to the next instruction address
        current_pc += 4;
    }

compile_exit:
    block_cycles_ = core->cycles - old_cycles;

    builder->CreateRetVoid(); // Return from the function

    // Prepare the module's LLVM IR
    std::string str;
    llvm::raw_string_ostream os(str);
    new_module->print(os, nullptr);
    os.flush();

    // Add the module to the LLJIT
    if (auto err = lljit->addIRModule(
            llvm::orc::ThreadSafeModule(std::move(new_module), std::make_unique<llvm::LLVMContext>())))
    {
        // Correctly pass the context
        Logger::error("Failed to add module to LLJIT: " + llvm::toString(std::move(err)));
        printf("Failed to add module to LLJIT: %s\n", llvm::toString(std::move(err)).c_str());
        fflush(stdout);
        return nullptr;
    }

    // Lookup the function in the JIT
    auto sym = lljit->lookup("exec_0x" + format("{:08X}", start_pc));
    if (!sym)
    {
        Logger::error("Failed to JIT compile function: " + llvm::toString(sym.takeError()));
        printf("Failed to JIT compile function: %s\n", llvm::toString(sym.takeError()).c_str());
        fflush(stdout);
        return nullptr;
    }

    auto exec_fn = (void (*)())sym->getValue(); // Use getValue() instead of getAddress()

    // Create and return the compiled block
    auto block = new CompiledBlock();
    block->start_pc = start_pc;
    block->end_pc = end_pc;
    block->code_ptr = (void *)exec_fn;
    block->last_used = execution_count;
    block->contains_branch = is_branch;
    block->llvm_ir = str;
    block->cycles = block_cycles_;

    return block;
}

void IOPJIT::initialize_opcode_table() {
    opcode_table[0x00].funct3_map[0x00] = &IOPJIT::iop_jit_sll; // SLL opcode
    opcode_table[0x00].funct3_map[0x08] = &IOPJIT::iop_jit_jr; // JR opcode
    opcode_table[0x00].funct3_map[0x21] = &IOPJIT::iop_jit_addu; // ADDU opcode
    opcode_table[0x00].funct3_map[0x25] = &IOPJIT::iop_jit_or; // OR opcode
    opcode_table[0x00].funct3_map[0x2B] = &IOPJIT::iop_jit_sltu; // SLTU opcode

    opcode_table[0x03].single_handler = &IOPJIT::iop_jit_jal; // JAL opcode
    opcode_table[0x04].single_handler = &IOPJIT::iop_jit_beq; // BEQ opcode
    opcode_table[0x05].single_handler = &IOPJIT::iop_jit_bne; // BNE opcode
    opcode_table[0x08].single_handler = &IOPJIT::iop_jit_addi; // ADDI opcode
    opcode_table[0x09].single_handler = &IOPJIT::iop_jit_addiu; // ADDIU opcode

    opcode_table[0x10].rs_map[0x00] = &IOPJIT::iop_jit_mfc0; // MFC0 opcode
    opcode_table[0x10].rs_map[0x04] = &IOPJIT::iop_jit_mtc0; // MTC0 opcode

    opcode_table[0x0A].single_handler = &IOPJIT::iop_jit_slti; // SLTI opcode
    opcode_table[0x0C].single_handler = &IOPJIT::iop_jit_andi; // ANDI opcode
    opcode_table[0x0D].single_handler = &IOPJIT::iop_jit_ori; // ORI opcode
    opcode_table[0x0F].single_handler = &IOPJIT::iop_jit_lui; // LUI opcode
    opcode_table[0x20].single_handler = &IOPJIT::iop_jit_lb;  // LB opcode
    opcode_table[0x23].single_handler = &IOPJIT::iop_jit_lw; // LW opcode
    opcode_table[0x28].single_handler = &IOPJIT::iop_jit_sb;  // SB opcode
    opcode_table[0x2B].single_handler = &IOPJIT::iop_jit_sw; // SW opcode
}

void IOPJIT::execute_cycles(uint64_t cycle_limit, Breakpoint *breakpoints)
{
    uint64_t current_cycles = 0;
    while (cycle_limit > current_cycles && !Neo2::is_aborted())
    {
        current_cycles += execute_block(breakpoints);
        core->registers[0] = 0;
    }
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
    Logger::error("Unknown IOP LLVM IR opcode: " + format("0x{:08X}", opcode) +
                    " at PC: 0x" + format("{:08X}", core->pc));
    Neo2::exit(1, Neo2::Subsystem::IOP);
}

void IOPJIT::step() {
    if (exec_type != RunType::Step)
    {
        block_cache.clear();
    }
    exec_type = RunType::Step;
    single_instruction_mode = true;
    if (!Neo2::is_aborted()) execute_block(nullptr);
    core->registers[0] = 0;
    single_instruction_mode = false;
}

void IOPJIT::run(Breakpoint *breakpoints)
{
    if (exec_type != RunType::Step)
    {
        block_cache.clear();
    }

    exec_type = RunType::Step;
    single_instruction_mode = false;
    if (!Neo2::is_aborted()) execute_block(breakpoints);
    core->registers[0] = 0;
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

void IOPJIT::iop_jit_addiu(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core)
{
    uint8_t rt = (opcode >> 16) & 0x1F;
    uint8_t rs = (opcode >> 21) & 0x1F;
    int16_t imm = static_cast<int16_t>(opcode & 0xFFFF); // Sign-extend immediate value

    llvm::Value *gpr_base = builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
                                                    llvm::PointerType::getUnqual(builder->getInt32Ty()));

    llvm::Value *rs_value = builder->CreateLoad(
        builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs)));

    // Sign-extend the immediate value to 64 bits
    llvm::Value *imm_value = builder->CreateSExt(builder->getInt32(imm), builder->getInt64Ty());

    llvm::Value *result = builder->CreateAdd(rs_value, imm_value);

    llvm::Value *rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt));

    builder->CreateStore(result, rt_ptr);
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
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
        builder->getInt32Ty(), gpr_base, builder->getInt32(rs)
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
    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt));
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

void IOPJIT::iop_jit_bne(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core)
{
    uint8_t rs = (opcode >> 21) & 0x1F;                        // Extract rs register
    uint8_t rt = (opcode >> 16) & 0x1F;                        // Extract rt register
    int16_t immediate = static_cast<int16_t>(opcode & 0xFFFF); // Extract immediate

    // Load the value from registers rs and rt
    llvm::Value *gpr_base = builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
                                                    llvm::PointerType::getUnqual(builder->getInt32Ty()));

    llvm::Value *rs_value = builder->CreateLoad(
        builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs)));
    llvm::Value *rt_value = builder->CreateLoad(
        builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt)));

    // Compare rs and rt (not equal)
    llvm::Value *comparison = builder->CreateICmpNE(rs_value, rt_value);

    // Calculate the branch target address
    llvm::Value *branch_target = builder->CreateAdd(
        builder->getInt32(current_pc),
        builder->getInt32(immediate * 4 + 4) // Branch offset (immediate * 4 + 4 for the next instruction)
    );

    // Update the PC based on the comparison result (branch taken or not)
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);

    // If branch is taken, update the branch destination and set branching to true
    builder->CreateStore(builder->CreateSelect(comparison, branch_target, builder->getInt32(current_pc + 4)),
                         builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)),
                                                 llvm::PointerType::getUnqual(builder->getInt32Ty())));
    builder->CreateStore(builder->CreateSelect(comparison, builder->getInt1(true), builder->getInt1(false)),
                         builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)),
                                                 llvm::PointerType::getUnqual(builder->getInt1Ty())));

    is_branch = true;
}

void IOPJIT::iop_jit_beq(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
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
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs))
    );

    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt))
    );

    // Create the condition: rs == rt (equality comparison)
    llvm::Value* condition = builder->CreateICmpEQ(rs_value, rt_value);

    // Calculate the branch target address (address of the instruction in the branch delay slot + (immediate * 4))
    llvm::Value* branch_target = builder->CreateAdd(
        builder->getInt32(current_pc + 4),  // Address of the instruction in the branch delay slot
        builder->CreateShl(builder->getInt32(immediate), 2)  // immediate * 4
    );

    // Execute the delay slot instruction, regardless of whether the branch is taken or not
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);

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

    EMIT_IOP_UPDATE_PC(core, builder, current_pc);

    // Set branch destination to the value in the RS register (jump address)
    builder->CreateStore(rs_value, builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty())));

    // Mark the processor as branching
    builder->CreateStore(builder->getInt1(true), builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty())));

    // Update the program counter and set is_branch to true
    is_branch = true;
}

void IOPJIT::iop_jit_jal(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core)
{
    // Extract the 26-bit target address from the opcode
    uint32_t target = opcode & 0x03FFFFFF;

    // The jump address is formed by shifting the target address left by 2 (to account for word alignment)
    // and setting the upper 4 bits of the address (from the current PC)
    uint32_t jump_address = (current_pc & 0xF0000000) | (target << 2);

    // Store the return address (current_pc + 4) into the $ra register (register 31)
    llvm::Value *gpr_base =
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
                                llvm::PointerType::getUnqual(builder->getInt32Ty()) // Each register is 32-bit
        );
    llvm::Value *return_address = builder->getInt32(current_pc + 8);
    builder->CreateStore(return_address, builder->CreateGEP(builder->getInt32Ty(), gpr_base,
                                                            builder->getInt32(31) // Store in $ra (register 31)
                                                            ));

    // Call EMIT_EE_UPDATE_PC to update the program counter (pc = jump_address)
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);

    // Update the program counter to the jump address
    builder->CreateStore(builder->getInt32(jump_address),
                         builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)),
                                                 llvm::PointerType::getUnqual(builder->getInt32Ty())));
    builder->CreateStore(builder->getInt1(true),
                         builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)),
                                                 llvm::PointerType::getUnqual(builder->getInt1Ty())));

    is_branch = true;
}

void IOPJIT::iop_jit_lw(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core)
{
    uint8_t rs = (opcode >> 21) & 0x1F;                     // Extract the base register (rs)
    uint8_t rt = (opcode >> 16) & 0x1F;                     // Extract the destination register (rt)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)

    llvm::Value *gpr_base = builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
                                                    llvm::PointerType::getUnqual(builder->getInt32Ty()));

    // Load the base register value (rs) from the GPR array
    llvm::Value *rs_value = builder->CreateLoad(
        builder->getInt64Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base,
                                                  builder->getInt32(rs) // GPR uses 32-bit (4 bytes) offset
                                                  ));

    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value *offset_value = builder->getInt32(offset);

    // Calculate the effective address: addr = base + offset
    llvm::Value *addr_value = builder->CreateAdd(rs_value, offset_value);

    // Call read32: read32(core, addr)
    llvm::Value *value_to_load = builder->CreateCall(
        iop_read32, {llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)), addr_value});

    // Store the zero-extended value in the rt register
    llvm::Value *rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt));
    builder->CreateStore(value_to_load, rt_ptr);

    // Emit the update to PC after this operation
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}

void IOPJIT::iop_jit_sw(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, IOP* core) {
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)

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

    // --- Cache isolation check ---
    // Assume the status register (SR) is stored in CP0 register 12.
    llvm::Value *sr_ptr =
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->cop0_registers[12])),
                                llvm::PointerType::getUnqual(builder->getInt32Ty()));
    llvm::Value *sr_value = builder->CreateLoad(builder->getInt32Ty(), sr_ptr);
    // Isolate the cache-isolation bit (0x10000).
    llvm::Value *isolation = builder->CreateAnd(sr_value, builder->getInt32(0x10000));
    // Condition: if (isolation == 0) then cache is not isolated and we can write.
    llvm::Value *not_isolated = builder->CreateICmpEQ(isolation, builder->getInt32(0));

    // Create two basic blocks: one for performing the write and one for skipping.
    llvm::Function *currentFunc = builder->GetInsertBlock()->getParent();
    llvm::BasicBlock *writeBB = llvm::BasicBlock::Create(builder->getContext(), "write", currentFunc);
    llvm::BasicBlock *skipBB = llvm::BasicBlock::Create(builder->getContext(), "skip", currentFunc);
    llvm::BasicBlock *continueBB = llvm::BasicBlock::Create(builder->getContext(), "continue", currentFunc);

    // Conditional branch based on cache isolation.
    builder->CreateCondBr(not_isolated, writeBB, skipBB);

    // In the "write" block, perform the store using iop_write8.
    builder->SetInsertPoint(writeBB);
    builder->CreateCall(iop_write32, {llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
                                     addr_value, value_to_store});
    // After writing, branch to the continue block.
    builder->CreateBr(continueBB);

    // In the "skip" block, optionally log a message (here we just call Logger::error).
    builder->SetInsertPoint(skipBB);
    // (You could emit a call to a logging function if desired; for now, we simply do nothing.)
    builder->CreateBr(continueBB);

    // Continue.
    builder->SetInsertPoint(continueBB);

    // Update the program counter.
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}

void IOPJIT::iop_jit_sb(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core)
{
    // Extract registers and offset.
    uint8_t rt = (opcode >> 16) & 0x1F;                     // Source register
    uint8_t rs = (opcode >> 21) & 0x1F;                     // Base register
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // 16-bit immediate

    // Get GPR base pointer (each register is 32-bit).
    llvm::Value *gpr_base = builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
                                                    llvm::PointerType::getUnqual(builder->getInt32Ty()));

    // Load base register (rs) value.
    llvm::Value *rs_value = builder->CreateLoad(
        builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs)));

    // Extend offset and compute effective address.
    llvm::Value *offset_value = builder->getInt32(offset);
    llvm::Value *addr_value = builder->CreateAdd(rs_value, offset_value);

    // Load value to store from register rt.
    llvm::Value *value_to_store = builder->CreateLoad(
        builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt)));

    // --- Cache isolation check ---
    // Assume the status register (SR) is stored in CP0 register 12.
    llvm::Value *sr_ptr =
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->cop0_registers[12])),
                                llvm::PointerType::getUnqual(builder->getInt32Ty()));
    llvm::Value *sr_value = builder->CreateLoad(builder->getInt32Ty(), sr_ptr);
    // Isolate the cache-isolation bit (0x10000).
    llvm::Value *isolation = builder->CreateAnd(sr_value, builder->getInt32(0x10000));
    // Condition: if (isolation == 0) then cache is not isolated and we can write.
    llvm::Value *not_isolated = builder->CreateICmpEQ(isolation, builder->getInt32(0));

    // Create two basic blocks: one for performing the write and one for skipping.
    llvm::Function *currentFunc = builder->GetInsertBlock()->getParent();
    llvm::BasicBlock *writeBB = llvm::BasicBlock::Create(builder->getContext(), "write", currentFunc);
    llvm::BasicBlock *skipBB = llvm::BasicBlock::Create(builder->getContext(), "skip", currentFunc);
    llvm::BasicBlock *continueBB = llvm::BasicBlock::Create(builder->getContext(), "continue", currentFunc);

    // Conditional branch based on cache isolation.
    builder->CreateCondBr(not_isolated, writeBB, skipBB);

    // In the "write" block, perform the store using iop_write8.
    builder->SetInsertPoint(writeBB);
    builder->CreateCall(iop_write8, {llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)),
                                     addr_value, value_to_store});
    // After writing, branch to the continue block.
    builder->CreateBr(continueBB);

    // In the "skip" block, optionally log a message (here we just call Logger::error).
    builder->SetInsertPoint(skipBB);
    // (You could emit a call to a logging function if desired; for now, we simply do nothing.)
    builder->CreateBr(continueBB);

    // Continue.
    builder->SetInsertPoint(continueBB);

    // Update the program counter.
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}

void IOPJIT::iop_jit_mtc0(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core)
{
    uint8_t rd = (opcode >> 11) & 0x1F;
    uint8_t rt = (opcode >> 16) & 0x1F;

    llvm::Value *gpr_base = builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
                                                    llvm::PointerType::getUnqual(builder->getInt32Ty()));
    llvm::Value *gpr_value = builder->CreateLoad(
        builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt)));
    llvm::Value *cop0_base =
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(core->cop0_registers)),
                                                     llvm::PointerType::getUnqual(builder->getInt32Ty()));
    llvm::Value *cop0_u32_0 = builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(rd));
    builder->CreateStore(gpr_value, cop0_u32_0);
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}

void IOPJIT::iop_jit_lb(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core)
{
    uint8_t rs = (opcode >> 21) & 0x1F;                     // Extract the base register (rs)
    uint8_t rt = (opcode >> 16) & 0x1F;                     // Extract the destination register (rt)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)

    llvm::Value *gpr_base =
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
                                llvm::PointerType::getUnqual(builder->getInt8Ty())
        );

    // Load the base register value (rs) from the GPR array
    llvm::Value *rs_value = builder->CreateLoad(
        builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base,
                                                  builder->getInt32(rs) // GPR uses 32-bit (4 bytes) offset
                                                  ));

    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value *offset_value = builder->getInt32(offset);

    // Calculate the effective address: addr = base + offset
    llvm::Value *addr_value = builder->CreateAdd(rs_value, offset_value);

    // Call read8: read8(core, addr)
    llvm::Value *value_to_load = builder->CreateCall(
        iop_read8, {llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)), addr_value});

    // Cast the loaded value to 8 bits to ensure correct zero-extension
    llvm::Value *byte_value = builder->CreateTrunc(value_to_load, builder->getInt8Ty());

    // Zero-extend the fetched byte
    llvm::Value *sign_extended_value = builder->CreateSExt(byte_value, builder->getInt32Ty());

    // Store the zero-extended value in the rt register
    llvm::Value *rt_ptr = builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt32(rt));
    builder->CreateStore(sign_extended_value, rt_ptr);

    // Emit the update to PC after this operation
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}


void IOPJIT::iop_jit_addu(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core)
{
    uint8_t rd = (opcode >> 11) & 0x1F; // Extract destination register (rd)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract first source register (rs)
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract second source register (rt)

    // Handle the GPR array
    llvm::Value *gpr_base =
        builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
                                llvm::PointerType::getUnqual(builder->getInt32Ty()) // Each register is 32-bit
        );

    // Load the values from the source registers (rs and rt)
    llvm::Value *rs_value = builder->CreateLoad(
        builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs)));

    llvm::Value *rt_value = builder->CreateLoad(
        builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt)));

    // Perform the addition (unsigned addition is the same as signed addition)
    llvm::Value *result = builder->CreateAdd(rs_value, rt_value);

    // Store the result into the destination register (rd)
    builder->CreateStore(result, builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt32(rd)));

    // Emit the update to PC after this operation
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}

void IOPJIT::iop_jit_sltu(std::uint32_t opcode, uint32_t &current_pc, bool &is_branch, IOP *core)
{
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract RT (bits 16-20)
    uint8_t rd = (opcode >> 11) & 0x1F; // Extract RD (bits 11-15)

    // Load the value from register GPR[rs]
    llvm::Value *gpr_base = builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
                                                    llvm::PointerType::getUnqual(builder->getInt32Ty()));

    llvm::Value *rs_value = builder->CreateLoad(
        builder->getInt32Ty(), builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt32(rs)));

    llvm::Value *rt_value = builder->CreateLoad(
        builder->getInt32Ty(), builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt32(rt)));

    // Perform unsigned comparison: GPR[rs] < immediate
    llvm::Value *condition = builder->CreateICmpULT(rs_value, rt_value);

    // Set GPR[rt] to 1 if condition is true, otherwise set to 0
    llvm::Value *result = builder->CreateSelect(condition, builder->getInt32(1), builder->getInt32(0));

    // Store the result into GPR[rt]
    builder->CreateStore(result, builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt32(rd)));

    // Update the program counter after the instruction is executed
    EMIT_IOP_UPDATE_PC(core, builder, current_pc);
}
