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

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

EEJIT::EEJIT(EE* core) : core(core) {
    // Initialize LLVM components
    llvm::InitializeNativeTarget();
    llvm::InitializeNativeTargetAsmPrinter();
    llvm::InitializeNativeTargetAsmParser();

    // Create context and builder
    context = std::make_unique<llvm::LLVMContext>();
    builder = std::make_unique<llvm::IRBuilder<>>(*context);

    // Create module
    module = std::make_unique<llvm::Module>("ee_jit", *context);
    
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
    
    ee_write32_type = llvm::FunctionType::get(builder->getInt32Ty(), {builder->getInt64Ty(), builder->getInt32Ty(), builder->getInt32Ty()}, false);
    ee_write32 = llvm::Function::Create(
    ee_write32_type, llvm::Function::ExternalLinkage, "ee_write32", module.get());;
    
    ready = true;
}

EEJIT::~EEJIT() {
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

void EEJIT::initialize_opcode_table() {
    opcode_table[0x00].funct3_map[0x00] = &EEJIT::ee_jit_sll; // SLL opcode
    opcode_table[0x00].funct3_map[0x08] = &EEJIT::ee_jit_jr; // JR opcode
    opcode_table[0x00].funct3_map[0x0F] = &EEJIT::ee_jit_sync; // SYNC opcode

    opcode_table[0x05].single_handler = &EEJIT::ee_jit_bne; // BNE opcode
    opcode_table[0x09].single_handler = &EEJIT::ee_jit_addiu; // ADDIU opcode

    opcode_table[0x10].rs_map[0x00] = &EEJIT::ee_jit_mfc0; // MFC0 opcode
    opcode_table[0x10].rs_map[0x04] = &EEJIT::ee_jit_mtc0; // MTC0 opcode
    opcode_table[0x10].rs_map[0x10] = &EEJIT::ee_jit_tlbwi; // TLBWI opcode

    opcode_table[0x0A].single_handler = &EEJIT::ee_jit_slti; // SLTI opcode
    opcode_table[0x0D].single_handler = &EEJIT::ee_jit_ori; // ORI opcode
    opcode_table[0x0F].single_handler = &EEJIT::ee_jit_lui; // LUI opcode
    opcode_table[0x2B].single_handler = &EEJIT::ee_jit_sw; // SW opcode
    opcode_table[0x42].single_handler = &EEJIT::ee_jit_srl; // SRL opcode
}

extern "C" void ee_write32(EE* core, uint32_t addr, uint32_t value) {
    core->bus->write32(addr, value);
}

std::tuple<bool, uint32_t, bool> EEJIT::generate_ir_for_opcode(uint32_t opcode, uint32_t current_pc) {
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

void EEJIT::base_error_handler(uint32_t opcode) {
    Logger::error("Unknown EE LLVM IR opcode: " + format("0x{:08X}", opcode));
    Neo2::exit(1, Neo2::Subsystem::EE);
}

void EEJIT::step() {
    single_instruction_mode = true;
    std::uint32_t opcode = core->fetch_opcode();
    execute_opcode(opcode);
    core->registers[0].u128 = 0;
    single_instruction_mode = false;
}

void EEJIT::run() {
    while (!Neo2::is_aborted()) {
        std::uint32_t opcode = core->fetch_opcode();
        execute_opcode(opcode);
        core->registers[0].u32[0] = 0;
    }
}

void EEJIT::execute_opcode(std::uint32_t opcode) {
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

CompiledBlock* EEJIT::compile_block(uint32_t start_pc, bool single_instruction) {
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
            Neo2::exit(1, Neo2::Subsystem::EE);
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

CompiledBlock* EEJIT::find_block(uint32_t pc) {
    auto it = block_cache.find(pc);
    if (it != block_cache.end()) {
        return &it->second;
    }
    return nullptr;
}

void EEJIT::evict_oldest_block() {
    if (lru_queue.empty()) return;
    uint32_t oldest_pc = lru_queue.front();
    lru_queue.erase(lru_queue.begin());
    block_cache.erase(oldest_pc);
}

void EEJIT::ee_jit_mfc0(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
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
    llvm::Value* gpr_u32_0 = builder->CreateGEP(builder->getInt32Ty(), gpr_base, {builder->getInt32(rt * 4)});
    builder->CreateStore(cop0_value, gpr_u32_0);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_mtc0(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rd = (opcode >> 11) & 0x1F;
    uint8_t rt = (opcode >> 16) & 0x1F;

    llvm::Value* gpr_base = builder->CreateIntToPtr( builder->getInt64(reinterpret_cast<uint64_t>(core->registers)), llvm::PointerType::getUnqual(builder->getInt32Ty()) );
    llvm::Value* gpr_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4)));
    llvm::Value* cop0_base = builder->CreateIntToPtr( builder->getInt64(reinterpret_cast<uint64_t>(core->cop0_registers)), llvm::PointerType::getUnqual(builder->getInt32Ty()) );
    llvm::Value* cop0_u32_0 = builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(rd));
    builder->CreateStore(gpr_value, cop0_u32_0);
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_addiu(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rt = (opcode >> 16) & 0x1F;
    uint8_t rs = (opcode >> 21) & 0x1F;
    int16_t imm = static_cast<int16_t>(opcode & 0xFFFF); // Sign-extend immediate value

    llvm::Value* gpr_base = builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(core->registers)), llvm::PointerType::getUnqual(builder->getInt32Ty()));
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)));
    llvm::Value* imm_value = builder->getInt32(imm);
    llvm::Value* result = builder->CreateAdd(rs_value, imm_value);
    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4));
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
    llvm::Value* result_int = builder->CreateZExt(result, builder->getInt32Ty());

    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4));
    builder->CreateStore(result_int, rt_ptr);

    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}

void EEJIT::ee_jit_bne(std::uint32_t opcode, uint32_t& current_pc, bool& is_branch, EE* core) {
    uint8_t rs = (opcode >> 21) & 0x1F;
    uint8_t rt = (opcode >> 16) & 0x1F;
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF);

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)));
    llvm::Value* rt_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4)));
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
    EMIT_EE_UPDATE_PC(core, builder, current_pc);

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
    llvm::Value* rt_ptr = builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4));
    builder->CreateStore(imm_value, rt_ptr);

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
    uint8_t rs = (opcode >> 21) & 0x1F;

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)));

    builder->CreateStore(rs_value, builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty())));
    builder->CreateStore(builder->getInt1(true), builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty())));

    EMIT_EE_UPDATE_PC(core, builder, current_pc);

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
    uint32_t index = core->cop0_registers[0];    // Index register
    uint32_t page_mask = core->cop0_registers[5]; // PageMask register
    uint32_t entry_hi = core->cop0_registers[10]; // EntryHi register
    uint32_t entry_lo0 = core->cop0_registers[2]; // EntryLo0 register
    uint32_t entry_lo1 = core->cop0_registers[3]; // EntryLo1 register

    // Logical AND of G bits
    bool g_bit = ((entry_lo0 & 0x01) & (entry_lo1 & 0x01)) != 0;

    // Create TLB entry
    TLBEntry entry;
    entry.page_mask = page_mask;
    entry.entry_hi = entry_hi;
    entry.entry_lo0 = (entry_lo0 & 0xFFFFFFFE) | (g_bit ? 0x01 : 0x00);
    entry.entry_lo1 = (entry_lo1 & 0xFFFFFFFE) | (g_bit ? 0x01 : 0x00);
    entry.global = g_bit;

    // Write to TLB entry at Index
    core->bus->tlb.write_entry(index, entry);

    // Update address space mappings based on the new TLB entry
    llvm::Value* address_space_r_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->bus->address_space_r)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    llvm::Value* address_space_w_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->bus->address_space_w)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );

    const uint32_t vpn2 = (entry_hi >> 13) & ~(page_mask >> 13);
    const uint32_t pfn0 = (entry_lo0 >> 6) & 0xFFFFF;
    const uint32_t pfn1 = (entry_lo1 >> 6) & 0xFFFFF;

    for (uint32_t i = 0; i < (page_mask >> 12) + 1; ++i) {
        llvm::Value* offset = builder->getInt32((vpn2 + i) * 8);
        llvm::Value* r_addr = builder->CreateGEP(builder->getInt64Ty(), address_space_r_base, offset);
        llvm::Value* w_addr = builder->CreateGEP(builder->getInt64Ty(), address_space_w_base, offset);

        uint64_t ram_ptr_val = reinterpret_cast<uint64_t>(&core->bus->ram[(pfn0 + i) * PAGE_SIZE]);
        llvm::Value* ram_ptr = builder->getInt64(ram_ptr_val);
        builder->CreateStore(ram_ptr, r_addr);
        if (!entry.global) {
            builder->CreateStore(ram_ptr, w_addr);
        }
    }

    for (uint32_t i = 0; i < (page_mask >> 12) + 1; ++i) {
        llvm::Value* offset = builder->getInt32((vpn2 + i + 1) * 8);
        llvm::Value* r_addr = builder->CreateGEP(builder->getInt64Ty(), address_space_r_base, offset);
        llvm::Value* w_addr = builder->CreateGEP(builder->getInt64Ty(), address_space_w_base, offset);

        uint64_t ram_ptr_val = reinterpret_cast<uint64_t>(&core->bus->ram[(pfn1 + i) * PAGE_SIZE]);
        llvm::Value* ram_ptr = builder->getInt64(ram_ptr_val);
        builder->CreateStore(ram_ptr, r_addr);
        if (!entry.global) {
            builder->CreateStore(ram_ptr, w_addr);
        }
    }

    // Update PC
    EMIT_EE_UPDATE_PC(core, builder, current_pc);
}
