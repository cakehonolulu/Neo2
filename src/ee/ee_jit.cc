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
#include <unistd.h>

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

    ee_write8_type = llvm::FunctionType::get(
        llvm::Type::getVoidTy(*context),
        {builder->getInt64Ty(),
        builder->getInt32Ty(),
        builder->getInt8Ty()},
        false
    );

    ee_write8 = llvm::Function::Create(
        ee_write8_type,
        llvm::Function::ExternalLinkage,
        "ee_write8", module.get()
    );

    ee_write16_type = llvm::FunctionType::get(
        llvm::Type::getVoidTy(*context),
        {builder->getInt64Ty(),
        builder->getInt32Ty(),
        builder->getInt16Ty()},
        false
    );

    ee_write16 = llvm::Function::Create(
        ee_write16_type,
        llvm::Function::ExternalLinkage,
        "ee_write16", module.get()
    );;
    
    ee_write32_type = llvm::FunctionType::get(
        llvm::Type::getVoidTy(*context),
        {builder->getInt64Ty(),
        builder->getInt32Ty(),
        builder->getInt32Ty()},
        false
    );

    ee_write32 = llvm::Function::Create(
        ee_write32_type,
        llvm::Function::ExternalLinkage,
        "ee_write32", module.get()
    );
    
    ee_write64_type = llvm::FunctionType::get(
        llvm::Type::getVoidTy(*context),
        {llvm::PointerType::getUnqual(builder->getInt8Ty()),
        builder->getInt32Ty(),
        builder->getInt64Ty()},
        false
    );

    ee_write64 = llvm::Function::Create(
        ee_write64_type,
        llvm::Function::ExternalLinkage,
        "ee_write64",
        module.get()
    );

    ee_write128_type = llvm::FunctionType::get(
        llvm::Type::getVoidTy(*context),
        {llvm::PointerType::getUnqual(builder->getInt8Ty()),
        builder->getInt32Ty(),
        builder->getInt128Ty()},
        false
    );

    ee_write128 = llvm::Function::Create(
        ee_write128_type,
        llvm::Function::ExternalLinkage,
        "ee_write128",
        module.get()
    );

    ee_read8_type = llvm::FunctionType::get(
        builder->getInt8Ty(),
        {llvm::PointerType::getUnqual(builder->getInt8Ty()),
        builder->getInt32Ty()},
        false
    );

    ee_read8 = llvm::Function::Create(
        ee_read8_type,
        llvm::Function::ExternalLinkage,
        "ee_read8",
        module.get()
    );

    ee_read16_type = llvm::FunctionType::get(
        builder->getInt16Ty(),
        {llvm::PointerType::getUnqual(builder->getInt8Ty()),
        builder->getInt32Ty()},
        false
    );

    ee_read16 = llvm::Function::Create(
        ee_read16_type,
        llvm::Function::ExternalLinkage,
        "ee_read16",
        module.get()
    );

    ee_read32_type = llvm::FunctionType::get(
        builder->getInt32Ty(),
        {llvm::PointerType::getUnqual(builder->getInt8Ty()),
        builder->getInt32Ty()},
        false
    );

    ee_read32 = llvm::Function::Create(
        ee_read32_type,
        llvm::Function::ExternalLinkage,
        "ee_read32",
        module.get()
    );

    ee_read64_type = llvm::FunctionType::get(
        builder->getInt64Ty(),
        {llvm::PointerType::getUnqual(builder->getInt8Ty()),
        builder->getInt32Ty()},
        false
    );

    ee_read64 = llvm::Function::Create(
        ee_read64_type,
        llvm::Function::ExternalLinkage,
        "ee_read64",
        module.get()
    );

    ee_read128_type = llvm::FunctionType::get(
        builder->getInt128Ty(),
        {llvm::PointerType::getUnqual(builder->getInt8Ty()),
        builder->getInt32Ty()},
        false
    );

    ee_read128 = llvm::Function::Create( 
        ee_read128_type,
        llvm::Function::ExternalLinkage,
        "ee_read128",
        module.get()
    );

    ee_tlb_write_type = llvm::FunctionType::get(
        llvm::Type::getVoidTy(*context),
        {llvm::PointerType::getUnqual(builder->getInt8Ty()),
        builder->getInt32Ty(),
        llvm::PointerType::getUnqual(builder->getInt8Ty())},
        false
    );

    ee_tlb_write = llvm::Function::Create(
        ee_tlb_write_type,
        llvm::Function::ExternalLinkage,
        "ee_tlb_write",
        module.get()
    );


    ee_write32_dbg_type = llvm::FunctionType::get(
        llvm::Type::getVoidTy(*context),
        {builder->getInt64Ty(),
        builder->getInt32Ty(),
        builder->getInt32Ty(),
        builder->getInt32Ty()},
        false
    );

    ee_write32_dbg = llvm::Function::Create(
        ee_write32_dbg_type,
        llvm::Function::ExternalLinkage,
        "ee_write32_dbg", module.get()
    );
    

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
    opcode_table[0x00].funct3_map[0x00] = {&EEJIT::ee_jit_sll, Default}; // SLL opcode
    opcode_table[0x00].funct3_map[0x02] = {&EEJIT::ee_jit_srl, Default}; // SRL opcode
    opcode_table[0x00].funct3_map[0x03] = {&EEJIT::ee_jit_sra, Default}; // SRA opcode
    opcode_table[0x00].funct3_map[0x08] = {&EEJIT::ee_jit_jr, Default}; // JR opcode
    opcode_table[0x00].funct3_map[0x09] = {&EEJIT::ee_jit_jalr, Default}; // JALR opcode
    opcode_table[0x00].funct3_map[0x0A] = {&EEJIT::ee_jit_movz, Default}; // MOVZ opcode
    opcode_table[0x00].funct3_map[0x0B] = {&EEJIT::ee_jit_movn, Default}; // MOVN opcode
    opcode_table[0x00].funct3_map[0x0F] = {&EEJIT::ee_jit_sync, Default}; // SYNC opcode
    opcode_table[0x00].funct3_map[0x10] = {&EEJIT::ee_jit_mfhi, Default}; // MFHI opcode
    opcode_table[0x00].funct3_map[0x12] = {&EEJIT::ee_jit_mflo, Default}; // MFLO opcode
    opcode_table[0x00].funct3_map[0x17] = {&EEJIT::ee_jit_dsrav, Default}; // DSRAV opcode
    opcode_table[0x00].funct3_map[0x18] = {&EEJIT::ee_jit_mult, Mult};  // MULT opcode
    opcode_table[0x00].funct3_map[0x1A] = {&EEJIT::ee_jit_div, Div}; // DIV opcode
    opcode_table[0x00].funct3_map[0x1B] = {&EEJIT::ee_jit_divu, Div}; // DIVU opcode
    opcode_table[0x00].funct3_map[0x21] = {&EEJIT::ee_jit_addu, Default}; // ADDU opcode
    opcode_table[0x00].funct3_map[0x23] = {&EEJIT::ee_jit_subu, Default}; // SUBU opcode
    opcode_table[0x00].funct3_map[0x24] = {&EEJIT::ee_jit_and, Default}; // AND opcode
    opcode_table[0x00].funct3_map[0x25] = {&EEJIT::ee_jit_or, Default}; // OR opcode
    opcode_table[0x00].funct3_map[0x2A] = {&EEJIT::ee_jit_slt, Default}; // SLT opcode
    opcode_table[0x00].funct3_map[0x2B] = {&EEJIT::ee_jit_sltu, Default}; // SLTU opcode
    opcode_table[0x00].funct3_map[0x2D] = {&EEJIT::ee_jit_dmove, Default}; // DMOVE opcode
    opcode_table[0x00].funct3_map[0x3C] = {&EEJIT::ee_jit_dsll32, Default}; // DSLL32 opcode
    opcode_table[0x00].funct3_map[0x3F] = {&EEJIT::ee_jit_dsra32, Default}; // DSRA32 opcode

    opcode_table[0x01].funct3_map[0x00] = {&EEJIT::ee_jit_bltz, Branch};  // BLTZ opcode
    opcode_table[0x01].funct3_map[0x01] = {&EEJIT::ee_jit_bgez, Branch};  // BGEZ opcode

    opcode_table[0x02].single_handler = {&EEJIT::ee_jit_j, Default}; // J opcode
    opcode_table[0x03].single_handler = {&EEJIT::ee_jit_jal, Default}; // JAL opcode
    opcode_table[0x04].single_handler = {&EEJIT::ee_jit_beq, Branch}; // BEQ opcode
    opcode_table[0x05].single_handler = {&EEJIT::ee_jit_bne, Branch}; // BNE opcode
    opcode_table[0x06].single_handler = {&EEJIT::ee_jit_blez, Branch}; // BLEZ opcode
    opcode_table[0x07].single_handler = {&EEJIT::ee_jit_bgtz, Branch}; // BGTZ opcode
    opcode_table[0x09].single_handler = {&EEJIT::ee_jit_addiu, Default}; // ADDIU opcode

    opcode_table[0x10].rs_map[0x00] = {&EEJIT::ee_jit_mfc0, CopDefault}; // MFC0 opcode
    opcode_table[0x10].rs_map[0x04] = {&EEJIT::ee_jit_mtc0, CopDefault}; // MTC0 opcode
    opcode_table[0x10].rs_map[0x10] = {&EEJIT::ee_jit_tlbwi, CopDefault}; // TLBWI opcode

    opcode_table[0x0A].single_handler = {&EEJIT::ee_jit_slti, Default}; // SLTI opcode
    opcode_table[0x0B].single_handler = {&EEJIT::ee_jit_sltiu, Default}; // SLTIU opcode
    opcode_table[0x0C].single_handler = {&EEJIT::ee_jit_andi, Default}; // ANDI opcode
    opcode_table[0x0D].single_handler = {&EEJIT::ee_jit_ori, Default}; // ORI opcode
    opcode_table[0x0E].single_handler = {&EEJIT::ee_jit_xori, Default}; // XORI opcode
    opcode_table[0x0F].single_handler = {&EEJIT::ee_jit_lui, Default}; // LUI opcode
    opcode_table[0x14].single_handler = {&EEJIT::ee_jit_beql, Branch}; // BEQL opcode
    opcode_table[0x15].single_handler = {&EEJIT::ee_jit_bnel, Branch}; // BNEL opcode

    opcode_table[0x1C].funct3_map[0x12] = {&EEJIT::ee_jit_mflo1, Default}; // MFLO1 opcode
    opcode_table[0x1C].funct3_map[0x18] = {&EEJIT::ee_jit_mult1, MMI_Mult}; // MULT1 opcode
    opcode_table[0x1C].funct3_map[0x1B] = {&EEJIT::ee_jit_divu1, MMI_Div}; // DIVU1 opcode

    opcode_table[0x20].single_handler = {&EEJIT::ee_jit_lb, Load};  // LB opcode
    opcode_table[0x23].single_handler = {&EEJIT::ee_jit_lw, Load}; // LW opcode
    opcode_table[0x24].single_handler = {&EEJIT::ee_jit_lbu, Load}; // LBU opcode
    opcode_table[0x25].single_handler = {&EEJIT::ee_jit_lhu, Load}; // LHU opcode
    opcode_table[0x28].single_handler = {&EEJIT::ee_jit_sb, Store}; // SB opcode
    opcode_table[0x29].single_handler = {&EEJIT::ee_jit_sh, Store}; // SH opcode
    opcode_table[0x2B].single_handler = {&EEJIT::ee_jit_sw, Store}; // SW opcode
    opcode_table[0x37].single_handler = {&EEJIT::ee_jit_ld, Load}; // LD opcode
    opcode_table[0x39].single_handler = {&EEJIT::ee_jit_swc1, Store}; // SWC1 opcode
    opcode_table[0x3F].single_handler = {&EEJIT::ee_jit_sd, Store}; // SD opcode
}

extern "C" void ee_write8(EE* core, uint32_t addr, uint8_t value) {
    core->bus->write8(addr, value);
}

extern "C" void ee_write16(EE* core, uint32_t addr, uint16_t value) {
    core->bus->write16(addr, value);
}

extern "C" void ee_write32(EE* core, uint32_t addr, uint32_t value) {
    core->bus->write32(addr, value);
}

extern "C" void ee_write64(EE* core, uint32_t addr, uint64_t value) {
    core->bus->write64(addr, value);
}

extern "C" void ee_write128(EE* core, uint32_t addr, uint128_t value) {
    core->bus->write128(addr, value);
}

extern "C" uint8_t ee_read8(EE* core, uint32_t addr) {
    return core->bus->read8(addr);
}

extern "C" uint16_t ee_read16(EE* core, uint32_t addr) {
    return core->bus->read16(addr);
}

extern "C" uint32_t ee_read32(EE* core, uint32_t addr) {
    return core->bus->read32(addr);
}

extern "C" uint64_t ee_read64(EE* core, uint32_t addr) {
    return core->bus->read64(addr);
}

extern "C" uint128_t ee_read128(EE* core, uint32_t addr) {
    return core->bus->read128(addr);
}

extern "C" void ee_tlb_write(EE* core, uint32_t index, const TLBEntry& entry) {
    core->bus->tlb.write_entry(index, entry);
}

extern "C" void ee_write32_dbg(EE* core, uint32_t addr, uint32_t value, uint32_t pc) {
    core->bus->write32_dbg(addr, value, pc);
}

std::tuple<bool, uint32_t, bool> EEJIT::generate_ir_for_opcode(uint32_t opcode, uint32_t current_pc) {
    bool is_branch = false;
    bool error = false;

    uint8_t opcode_index = (opcode >> 26) & 0x3F;
    uint8_t funct_or_rs = (opcode_index == 0x10) ? ((opcode >> 21) & 0x1F) : (opcode & 0x3F);
    uint8_t funct3 = (opcode >> 21) & 0x07;

    auto it = opcode_table.find(opcode_index);
    if (it != opcode_table.end()) {
        OpcodeHandler handler = nullptr;

        if (opcode_index == 0x01) {  // Handling branch opcodes
            uint8_t rt = (opcode >> 16) & 0x1F;
            auto funct_it = it->second.funct3_map.find(rt);
            if (funct_it != it->second.funct3_map.end()) {
                handler = funct_it->second.first;
                core->cycles += funct_it->second.second;
            }
        } else if (opcode_index == 0x10) {  // Special case for MIPS coprocessor instructions (like MFC0, MTC0, etc.)
            auto rs_it = it->second.rs_map.find(funct_or_rs);
            if (rs_it != it->second.rs_map.end()) {
                handler = rs_it->second.first;
                core->cycles = rs_it->second.second;
            }
        } else if (opcode_index == 0x1C) {  // Special case for MMI instructions
            // Get low 6 bits of opcode
            std::uint32_t funct = opcode & 0x3F;
            auto funct_it = it->second.funct3_map.find(funct);
            if (funct_it != it->second.funct3_map.end()) {
                handler = funct_it->second.first;
                core->cycles = funct_it->second.second;
            }
        } else {
            // Handle regular opcodes
            auto funct_it = it->second.funct3_map.find(funct_or_rs);
            if (funct_it != it->second.funct3_map.end())
            {
                handler = funct_it->second.first;
                core->cycles = funct_it->second.second;
            } else {
                handler = it->second.single_handler.first;
                core->cycles = it->second.single_handler.second;
            }
        }

        if (handler) {
            (this->*(handler))(opcode, current_pc, is_branch, core);
        } else {
            base_error_handler(opcode, current_pc);
        }
    } else {
        base_error_handler(opcode, current_pc);
    }

    return {is_branch, current_pc, error};
}

void EEJIT::base_error_handler(uint32_t opcode, uint32_t pc) {
    Logger::error("Unknown EE LLVM IR opcode: " + format("0x{:08X}", opcode) +
                    " at PC: 0x" + format("{:08X}", pc));
    Neo2::exit(1, Neo2::Subsystem::EE);
}

void EEJIT::step() {
    single_instruction_mode = true;
    execute_opcode();
    core->registers[0].u128 = 0;
    single_instruction_mode = false;
}

void EEJIT::run() {
    single_instruction_mode = false;
    while (!Neo2::is_aborted()) {
        execute_opcode_();
        core->registers[0].u128 = 0;
    }
}

void EEJIT::execute_opcode_() {
    // Try to find existing block
    CompiledBlock* block = find_block(core->pc);
    
    if (!block) {
        // Compile new block if not found
        block = compile_block_(core->pc, single_instruction_mode);
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
    core->cycles += block->cycles;
    int result = exec_fn();
}

CompiledBlock* EEJIT::compile_block_(uint32_t start_pc, bool single_instruction) {
    uint32_t current_pc = start_pc;
    uint32_t end_pc = start_pc; // Initialize end_pc to start_pc
    bool is_branch = false;
    uint32_t block_cycles = 0;
    
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

    uint64_t old_cycles = core->cycles;
    uint64_t block_cycles_ = 0;

    while (!Neo2::is_aborted()) {
        uint32_t opcode = core->fetch_opcode(current_pc); // Fetch the next opcode

        // Generate IR for the opcode
        auto [branch, current_pc_, error] = generate_ir_for_opcode(opcode, current_pc);

        is_branch = branch;

        if (error) {
            Logger::error("Error generating IR for opcode");
            Neo2::exit(1, Neo2::Subsystem::EE);
            return nullptr;
        }

        // Check if opcode is a branch or jump
        if (single_instruction || is_branch) {

            if (!single_instruction)
            {
                uint32_t func = (opcode >> 26) & 0x3F;

                if (func == 0x14 || func == 0x15) goto likely_detection;

                current_pc += 4;
                opcode = core->fetch_opcode(current_pc);

                auto [branch, current_pc_, error] = generate_ir_for_opcode(opcode, current_pc);

                if (error) {
                    Logger::error("Error generating IR for branch-delay opcode");
                    Neo2::exit(1, Neo2::Subsystem::EE);
                    return nullptr;
                }
            }

likely_detection:
            end_pc = current_pc_;
            break;
        }

        current_pc = current_pc_;  // Update the PC to the next instruction address

        if (!single_instruction) current_pc += 4;
    }

    block_cycles_ = core->cycles - old_cycles;

    builder->CreateRetVoid();  // Return from the function

    // Prepare the module's LLVM IR
    std::string str;
    llvm::raw_string_ostream os(str);
    new_module->print(os, nullptr);
    os.flush();

    // Add the module to the execution engine
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

    // Create and return the compiled block
    auto block = new CompiledBlock();
    block->start_pc = start_pc;
    block->end_pc = end_pc;
    block->code_ptr = (void*)exec_fn;
    block->last_used = execution_count;
    block->contains_branch = is_branch;
    block->llvm_ir = str;
    block->cycles = block_cycles_;

    return block;
}

void EEJIT::execute_opcode() {
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
    core->cycles += block->cycles;
    int result = exec_fn();
}

CompiledBlock* EEJIT::compile_block(uint32_t start_pc, bool single_instruction) {
    uint32_t current_pc = start_pc;
    uint32_t end_pc = start_pc; // Initialize end_pc to start_pc
    bool is_branch = false;
    uint32_t block_cycles = 0;
    
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

    uint64_t old_cycles = core->cycles;
    uint64_t block_cycles_ = 0;

    while (true) {
        uint32_t opcode = core->fetch_opcode(); // Fetch the next opcode
        
        // Generate IR for the opcode
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

        current_pc = current_pc_;  // Update the PC to the next instruction address
    }

    block_cycles_ = core->cycles - old_cycles;

    builder->CreateRetVoid();  // Return from the function

    // Prepare the module's LLVM IR
    std::string str;
    llvm::raw_string_ostream os(str);
    new_module->print(os, nullptr);
    os.flush();

    // Add the module to the execution engine
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

    // Create and return the compiled block
    auto block = new CompiledBlock();
    block->start_pc = start_pc;
    block->end_pc = end_pc;
    block->code_ptr = (void*)exec_fn;
    block->last_used = execution_count;
    block->contains_branch = is_branch;
    block->llvm_ir = str;
    block->cycles = block_cycles_;

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

    // Load the cycles value from core->cycles
    llvm::Value* cycles_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&(core->cycles))),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    llvm::Value* cycles_value = builder->CreateLoad(builder->getInt64Ty(), cycles_ptr);

    llvm::Value* offset_cycles = builder->CreateAdd(cycles_value, builder->getInt64(11));

    // Store the cycles value to cop0_registers[9]
    llvm::Value* cop0_reg9_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&(core->cop0_registers[9]))),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    builder->CreateStore(offset_cycles, cop0_reg9_ptr);

    llvm::Value* cop0_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->cop0_registers)),
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
        builder->getInt64(reinterpret_cast<uint64_t>(&(core->cop0_registers[9]))),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );
    llvm::Value* cop0_reg9 = builder->CreateLoad(builder->getInt32Ty(), cop0_reg9_ptr);
    llvm::Value* updated_cop0_reg9 = builder->CreateAdd(cop0_reg9, builder->getInt32(7));
    builder->CreateStore(updated_cop0_reg9, cop0_reg9_ptr);

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
    uint8_t rs = (opcode >> 21) & 0x1F;

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    llvm::Value* rs_value = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rs * 4)));

    EMIT_EE_UPDATE_PC(core, builder, current_pc);

    // Set branch destination to the value in the RS register (jump address)
    builder->CreateStore(rs_value, builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)), llvm::PointerType::getUnqual(builder->getInt32Ty())));

    // Mark the processor as branching
    builder->CreateStore(builder->getInt1(true), builder->CreateIntToPtr(builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)), llvm::PointerType::getUnqual(builder->getInt1Ty())));

    // Update the program counter and set is_branch to true
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
        builder->getInt64(reinterpret_cast<uint64_t>(core->cop0_registers)),
        llvm::PointerType::getUnqual(builder->getInt32Ty())
    );

    // Load core->cop0_registers[0] (Index register)
    llvm::Value* index = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(0)));

    // Load core->cop0_registers[5] (PageMask register)
    llvm::Value* page_mask = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(5)));

    // Load core->cop0_registers[10] (EntryHi register)
    llvm::Value* entry_hi = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(10)));

    // Load core->cop0_registers[2] (EntryLo0 register)
    llvm::Value* entry_lo0 = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(2)));

    // Load core->cop0_registers[3] (EntryLo1 register)
    llvm::Value* entry_lo1 = builder->CreateLoad(builder->getInt32Ty(), builder->CreateGEP(builder->getInt32Ty(), cop0_base, builder->getInt32(3)));

    // Logical AND of G bits
    llvm::Value* g_bit = builder->CreateAnd(entry_lo0, builder->getInt32(0x01));
    g_bit = builder->CreateAnd(g_bit, entry_lo1);
    g_bit = builder->CreateICmpNE(g_bit, builder->getInt32(0));  // g_bit is true if non-zero

    // Truncate g_bit to i1 (boolean) before storing it into the struct
    g_bit = builder->CreateTrunc(g_bit, builder->getInt1Ty());

    llvm::StructType* TLBEntryType = llvm::StructType::create(*context, "TLBEntry");
    llvm::Type* TLBEntryFields[] = {
        builder->getInt32Ty(),
        builder->getInt32Ty(),
        builder->getInt32Ty(),
        builder->getInt32Ty(),
        builder->getInt1Ty()
    };
    TLBEntryType->setBody(TLBEntryFields);

    // Create a TLB entry value
    llvm::Value* entry = builder->CreateAlloca(TLBEntryType);

    // Assign fields to the entry
    llvm::Value* entry_lo0_val = builder->CreateOr(entry_lo0, builder->CreateSelect(g_bit, builder->getInt32(0x01), builder->getInt32(0x00)));
    llvm::Value* entry_lo1_val = builder->CreateOr(entry_lo1, builder->CreateSelect(g_bit, builder->getInt32(0x01), builder->getInt32(0x00)));

    // Set the fields of the entry
    builder->CreateStore(page_mask, builder->CreateStructGEP(TLBEntryType, entry, 0)); // page_mask
    builder->CreateStore(entry_hi, builder->CreateStructGEP(TLBEntryType, entry, 1));  // entry_hi
    builder->CreateStore(entry_lo0_val, builder->CreateStructGEP(TLBEntryType, entry, 2)); // entry_lo0
    builder->CreateStore(entry_lo1_val, builder->CreateStructGEP(TLBEntryType, entry, 3)); // entry_lo1
    builder->CreateStore(g_bit, builder->CreateStructGEP(TLBEntryType, entry, 4));  // global bit

    builder->CreateCall(ee_tlb_write, {
        llvm::ConstantInt::get(builder->getInt64Ty(),
        reinterpret_cast<uint64_t>(core)),
        index,
        entry
    });

    // Update address space mappings based on the new TLB entry
    /*llvm::Value* address_space_r_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->bus->address_space_r)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    llvm::Value* address_space_w_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->bus->address_space_w)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );

    // Calculate vpn2, pfn0, pfn1 dynamically with LLVM
    llvm::Value* vpn2 = builder->CreateLShr(entry_hi, builder->getInt32(13));  // vpn2 = (entry_hi >> 13)
    vpn2 = builder->CreateAnd(vpn2, builder->CreateNot(builder->CreateLShr(page_mask, builder->getInt32(13)))); // vpn2 &= ~(page_mask >> 13)

    llvm::Value* pfn0 = builder->CreateLShr(entry_lo0, builder->getInt32(6));  // pfn0 = (entry_lo0 >> 6)
    pfn0 = builder->CreateAnd(pfn0, builder->getInt32(0xFFFFF));  // pfn0 &= 0xFFFFF

    llvm::Value* pfn1 = builder->CreateLShr(entry_lo1, builder->getInt32(6));  // pfn1 = (entry_lo1 >> 6)
    pfn1 = builder->CreateAnd(pfn1, builder->getInt32(0xFFFFF));  // pfn1 &= 0xFFFFF

    // Number of entries to loop over based on page_mask
    llvm::Value* num_entries = builder->CreateAdd(builder->CreateLShr(page_mask, builder->getInt32(12)), builder->getInt32(1)); // (page_mask >> 12) + 1

    // Dynamically generated address space updates using LLVM control flow
    llvm::Value* i = builder->CreateAlloca(builder->getInt32Ty(), nullptr, "i"); // Allocate space for i
    builder->CreateStore(builder->getInt32(0), i);  // Initialize i to 0

    llvm::BasicBlock* loop_header = llvm::BasicBlock::Create(*context, "loop_header", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* loop_body = llvm::BasicBlock::Create(*context, "loop_body", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* loop_end = llvm::BasicBlock::Create(*context, "loop_end", builder->GetInsertBlock()->getParent());

    builder->CreateBr(loop_header);
    builder->SetInsertPoint(loop_header);

    // Load the current value of i and compare it with num_entries
    llvm::Value* i_val = builder->CreateLoad(builder->getInt32Ty(), i, "i_val");
    llvm::Value* cond = builder->CreateICmpSLT(i_val, num_entries);  // i < num_entries
    builder->CreateCondBr(cond, loop_body, loop_end);  // If true, go to loop_body; otherwise, exit

    builder->SetInsertPoint(loop_body);

    // Correctly calculate the address by adding pfn0 + i and then multiplying by PAGE_SIZE
    llvm::Value* pfn0_plus_i = builder->CreateAdd(pfn0, i_val);  // pfn0 + i
    llvm::Value* offset = builder->CreateMul(pfn0_plus_i, builder->getInt32(PAGE_SIZE)); // (pfn0 + i) * PAGE_SIZE

    llvm::Value* r_addr = builder->CreateGEP(builder->getInt64Ty(), address_space_r_base, offset);
    llvm::Value* w_addr = builder->CreateGEP(builder->getInt64Ty(), address_space_w_base, offset);

    llvm::Value* ram_ptr_val = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->bus->ram[0])),
        llvm::PointerType::getUnqual(builder->getInt8Ty()) // We need a pointer to the memory location (int8 or appropriate type)
    );

    ram_ptr_val = builder->CreateGEP(builder->getInt8Ty(), ram_ptr_val, offset);  // Add offset to the base RAM address
    builder->CreateStore(ram_ptr_val, r_addr);

    llvm::Value* condition = builder->CreateICmpEQ(g_bit, llvm::ConstantInt::get(builder->getInt1Ty(), 0)); // g_bit == 0 (false)

    // Create a basic block for the conditional store
    llvm::BasicBlock* store_block = llvm::BasicBlock::Create(*context, "store_block", builder->GetInsertBlock()->getParent());
    llvm::BasicBlock* after_store_block = llvm::BasicBlock::Create(*context, "after_store_block", builder->GetInsertBlock()->getParent());

    // Conditionally branch based on g_bit
    builder->CreateCondBr(condition, store_block, after_store_block);

    // Insert store operation in the store_block
    builder->SetInsertPoint(store_block);
    builder->CreateStore(ram_ptr_val, w_addr);  // Store only if g_bit is 0
    builder->CreateBr(after_store_block);  // Continue after the store block

    // Insert the code after the store (if no store occurs, control will flow here)
    builder->SetInsertPoint(after_store_block);

    // Increment `i`
    llvm::Value* next_i = builder->CreateAdd(i_val, builder->getInt32(1));
    builder->CreateStore(next_i, i);  // Store the incremented value back into i
    builder->CreateBr(loop_header);

    builder->SetInsertPoint(loop_end);*/

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
    uint8_t rt = (opcode >> 16) & 0x1F; // Extract the destination register (rt)
    uint8_t rs = (opcode >> 21) & 0x1F; // Extract the base register (rs)
    int16_t offset = static_cast<int16_t>(opcode & 0xFFFF); // Extract the offset (16-bit immediate)
    
    // Handle 128-bit register access for rs (base register)
    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty()) // Each register is 64-bit for SD
    );
    
    // Load the base register value (rs) from the GPR array
    llvm::Value* rs_value = builder->CreateLoad(builder->getInt64Ty(), builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt64(rs * 2) // GPR uses 64-bit
    ));
    
    // Offset is a 16-bit signed immediate, so extend it to 32-bit
    llvm::Value* offset_value = builder->getInt32(offset);
    
    // Calculate the effective address: addr = base + offset
    llvm::Value* addr_value = builder->CreateAdd(rs_value, offset_value);
    
    // Load the value to store from register rt (64-bit value)
    llvm::Value* value_to_store = builder->CreateLoad(builder->getInt64Ty(), builder->CreateGEP(
        builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2) // GPR uses 64-bit
    ));
    
    // Call write64: write64(core, addr, value_to_store)
    builder->CreateCall(ee_write64, {
        llvm::ConstantInt::get(builder->getInt64Ty(), reinterpret_cast<uint64_t>(core)), 
        addr_value, value_to_store
    });
    
    // Emit the update to PC after this operation
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

    // Perform unsigned division
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
        branch_dest,
        builder->CreateAdd(builder->getInt64(current_pc), builder->getInt64(8)) // Increment PC by 4 if no branch
    );

    builder->CreateStore(pc_new, pc_ptr);

    // Update branching metadata
    llvm::Value* branching_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)),
        llvm::PointerType::getUnqual(builder->getInt1Ty())
    );
    builder->CreateStore(condition, branching_ptr);

    llvm::Value* branch_dest_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    builder->CreateStore(branch_dest, branch_dest_ptr);
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
    
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt32Ty(),
        builder->CreateGEP(builder->getInt32Ty(), gpr_base, builder->getInt32(rt * 4))
    );
    
    // Inequality comparison (rs != rt)
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
        branch_dest,
        builder->CreateAdd(builder->getInt64(current_pc), builder->getInt64(8)) // Increment PC by 4 if no branch
    );

    builder->CreateStore(pc_new, pc_ptr);

    // Update branching metadata
    llvm::Value* branching_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branching)),
        llvm::PointerType::getUnqual(builder->getInt1Ty())
    );
    builder->CreateStore(condition, branching_ptr);

    llvm::Value* branch_dest_ptr = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(&core->branch_dest)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );
    builder->CreateStore(branch_dest, branch_dest_ptr);
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
    uint8_t rs = (opcode >> 21) & 0x1F;  // Extract RS (bits 21-25)
    uint8_t rt = (opcode >> 16) & 0x1F;  // Extract RT (bits 16-20)
    uint8_t rd = (opcode >> 11) & 0x1F;  // Extract RD (bits 11-15)

    llvm::Value* gpr_base = builder->CreateIntToPtr(
        builder->getInt64(reinterpret_cast<uint64_t>(core->registers)),
        llvm::PointerType::getUnqual(builder->getInt64Ty())
    );

    // Load the value from register GPR[rs]
    llvm::Value* rs_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rs * 2))  // 64-bit values, so each register is 8 bytes
    );

    // Load the value from register GPR[rt]
    llvm::Value* rt_value = builder->CreateLoad(
        builder->getInt64Ty(),
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rt * 2))
    );

    // Perform bitwise AND: GPR[rs] AND GPR[rt]
    llvm::Value* result = builder->CreateAnd(rs_value, rt_value);

    // Store the result into GPR[rd]
    builder->CreateStore(
        result,
        builder->CreateGEP(builder->getInt64Ty(), gpr_base, builder->getInt64(rd * 2))
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
