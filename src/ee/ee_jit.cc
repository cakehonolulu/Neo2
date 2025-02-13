#include "cpu/breakpoint.hh"
#include <ee/ee_jit.hh>
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
#include <thread>

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

EEJIT::~EEJIT() {
    if (builder) {
        builder.reset();
    }

    if (context) {
        context.reset();
    }
}

std::tuple<bool, uint32_t, bool> EEJIT::generate_ir_for_opcode(uint32_t opcode, uint32_t current_pc) {
    bool is_branch = false;
    bool error = false;

    uint8_t opcode_index = (opcode >> 26) & 0x3F;

    uint8_t funct_or_rs = (opcode_index == 0x10 | opcode_index == 0x11 | opcode_index == 0x12) ? ((opcode >> 21) & 0x1F) : (opcode & 0x3F);

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
        } else if (opcode_index == 0x10 || opcode_index == 0x11 || opcode_index == 0x12) {  // Special case for MIPS coprocessor instructions (like MFC0, MTC0, etc.)
            if (opcode_index == 0x11) {
                //Logger::info("COP1 opcode");
                handler = &EEJIT::ee_jit_cop2;
                core->cycles += 7;
            } else if (opcode_index == 0x12) {
                //Logger::info("COP2 opcode");
                handler = &EEJIT::ee_jit_cop2;
                core->cycles += 7;
            } else {  // COP0 instructions
                // Check if RS maps to a primary COP0 instruction (e.g., MFC0, MTC0)
                auto rs_it = opcode_table[0x10].rs_map.find(funct_or_rs);
                if (rs_it != opcode_table[0x10].rs_map.end()) {
                    handler = rs_it->second.first;
                    core->cycles += rs_it->second.second;
                } else {
                    // Handle COP0 sub-opcodes (e.g., TLBWI)
                    uint8_t funct = opcode & 0x3F;  // Extract the sub-opcode (function code)
                    auto subfunc_it = opcode_table[0x10].subfunc_map[0x10].find(funct);
                    if (subfunc_it != opcode_table[0x10].subfunc_map[0x10].end()) {
                        handler = subfunc_it->second.first;
                        core->cycles += subfunc_it->second.second;
                    } else {
                        base_error_handler(opcode, current_pc);  // Unknown COP0 sub-opcode
                    }
                }
            }
        } else if (opcode_index == 0x1C) {  // Special case for MMI instructions
            // Get low 6 bits of opcode
            std::uint32_t funct = opcode & 0x3F;
            auto funct_it = it->second.funct3_map.find(funct);
            if (funct_it != it->second.funct3_map.end()) {
                handler = funct_it->second.first;
                core->cycles += funct_it->second.second;
            } else {
                uint32_t subfunc = (opcode >> 6) & 0x1F; // Adjust based on your sub-opcode extraction logic
                auto subfunc_it = it->second.subfunc_map[funct].find(subfunc);
                if (subfunc_it != it->second.subfunc_map[funct].end()) {
                    handler = subfunc_it->second.first;
                    core->cycles += subfunc_it->second.second;
                }
            }
        } else {
            // Handle regular opcodes
            auto funct_it = it->second.funct3_map.find(funct_or_rs);
            if (funct_it != it->second.funct3_map.end())
            {
                handler = funct_it->second.first;
                core->cycles += funct_it->second.second;
            } else {
                handler = it->second.single_handler.first;
                core->cycles += it->second.single_handler.second;
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
    if (exec_type != RunType::Step)
    {
        block_cache.clear();
    }
    exec_type = RunType::Step;
    single_instruction_mode = true;
    execute_block(nullptr);
    core->registers[0].u128 = 0;
    single_instruction_mode = false;
}

void EEJIT::run(Breakpoint *breakpoints) {
    if (exec_type != RunType::Step)
    {
        block_cache.clear();
    }

    if (!Neo2::is_aborted()) core->bus->gs.simul_vblank();
    
    exec_type = RunType::Step;
    single_instruction_mode = false;
    if (!Neo2::is_aborted()) execute_block(breakpoints);
    core->registers[0].u128 = 0;
}

void EEJIT::execute_cycles(uint64_t cycle_limit, Breakpoint *breakpoints) {
    uint64_t current_cycles = 0;
    while (cycle_limit > current_cycles && !Neo2::is_aborted()) {
        current_cycles += execute_block(breakpoints);
        core->registers[0].u128 = 0;
    }
}

uint32_t EEJIT::execute_block(Breakpoint *breakpoints) {
    // Try to find existing block
    CompiledBlock* block = find_block(core->pc);

    if (!block) {
        // Compile a new block if not found
        block = compile_block(core->pc, breakpoints);
        if (!block) {
            Logger::error("Error compiling block!");
            return Neo2::exit(1, Neo2::Subsystem::EE);
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
    exec_fn();
    return block->cycles;
}

CompiledBlock* EEJIT::compile_block(uint32_t start_pc, Breakpoint *breakpoints) {
    uint32_t current_pc = start_pc;
    uint32_t end_pc = start_pc; // Initialize end_pc to start_pc
    bool is_branch = false;

    // Create a new module for this block
    auto new_module = std::make_unique<llvm::Module>(
        "block_0x" + format("{:08X}", start_pc), *context);

    if (!new_module) {
        Logger::error("Failed to create LLVM module");
        return nullptr;
    }

    // Create function and basic block
    llvm::FunctionType *funcType = llvm::FunctionType::get(builder->getInt32Ty(), false);
    llvm::Function *func = llvm::Function::Create(
        funcType,
        llvm::Function::ExternalLinkage,
        "exec_0x" + format("{:08X}", start_pc),
        new_module.get()
    );

    llvm::BasicBlock *entry = llvm::BasicBlock::Create(*context, "entry", func);
    builder->SetInsertPoint(entry);

    uint64_t old_cycles = core->cycles;
    uint64_t block_cycles_ = 0;

    setup_ee_jit_primitives(new_module); // Pass new_module to setup_ee_jit_primitives

    while (!Neo2::is_aborted()) {
        if (breakpoints && breakpoints->has_breakpoint(current_pc, CoreType::EE)) {
            // Notify the main thread
            Neo2::pause_emulation();

            // Set breakpoint information in the debug interface
            breakpoints->notify_breakpoint(current_pc);

            goto compile_exit;
        }

        if (core->branching) {
            if (!core->likely_branch && single_instruction_mode) goto cont;

            if (single_instruction_mode)
            {
                current_pc = core->pc;

                if (core->branching && core->likely_branch)
                {
                    goto cont;
                }
            }
            
            if (!single_instruction_mode) core->branching = false; // Reset branching state
            uint32_t delay_slot_pc = current_pc;
            // Process the branch delay slot instruction
            uint32_t opcode = core->fetch_opcode(delay_slot_pc);
            auto [branch, _, error] = generate_ir_for_opcode(opcode, delay_slot_pc);
            if (error) {
                Logger::error("Error processing likely delay slot opcode");
                Neo2::exit(1, Neo2::Subsystem::EE);
                return nullptr;
            }

            current_pc = core->branch_dest + 4; // Set PC to the branch destination
        }

cont:
        uint32_t opcode = core->fetch_opcode(current_pc); // Fetch the next opcode

        // Generate IR for the opcode
        auto [branch, current_pc_, error] = generate_ir_for_opcode(opcode, current_pc);

        is_branch = branch;

        if (error) {
            Logger::error("Error generating IR for opcode");
            Neo2::exit(1, Neo2::Subsystem::EE);
            return nullptr;
        }

        // Check if the opcode is a branch or jump
        if (is_branch || single_instruction_mode) {
            uint32_t func = (opcode >> 26) & 0x3F;

            if ((func == 0x14 || func == 0x15))
            {
                core->likely_branch = true;
                goto compile_exit;
            }

            if ((func == 0x10 && ((opcode & 0x3F) == 0x18)))
            {
                core->likely_branch = true;
                goto compile_exit;
            }

            if ((func == 0x01 && (((opcode >> 16) & 0x1F)) == 0x02))
            {
                core->likely_branch = true;
                goto compile_exit;
            }

            if ((func == 0x00 && ((opcode & 0x3F) == 0x0C)))
            {
                core->likely_branch = true;
                goto compile_exit;
            }

            if (!single_instruction_mode)
            {
                current_pc += 4;
                opcode = core->fetch_opcode(current_pc);

                auto [branch, current_pc_, error] = generate_ir_for_opcode(opcode, current_pc);

                if (error) {
                    Logger::error("Error generating IR for branch-delay opcode");
                    Neo2::exit(1, Neo2::Subsystem::EE);
                    return nullptr;
                }
            }
            
            end_pc = current_pc_;
            break;
        }

        current_pc = current_pc_;  // Update PC to the next instruction address
        current_pc += 4;
    }

compile_exit:
    block_cycles_ = core->cycles - old_cycles;

    builder->CreateRetVoid();  // Return from the function

    // Prepare the module's LLVM IR
    std::string str;
    llvm::raw_string_ostream os(str);
    new_module->print(os, nullptr);
    os.flush();

    // Add the module to the LLJIT
    if (auto err = lljit->addIRModule(llvm::orc::ThreadSafeModule(std::move(new_module), std::make_unique<llvm::LLVMContext>()))) {
        // Correctly pass the context
        Logger::error("Failed to add module to LLJIT: " + llvm::toString(std::move(err)));
        printf("Failed to add module to LLJIT: %s\n", llvm::toString(std::move(err)).c_str());
        fflush(stdout);
        return nullptr;
    }

    // Lookup the function in the JIT
    auto sym = lljit->lookup("exec_0x" + format("{:08X}", start_pc));
    if (!sym) {
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
