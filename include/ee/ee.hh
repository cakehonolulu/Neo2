#pragma once

#include <bus/bus.hh>
#include <cpu/cpu.hh>
#include <ee/vu/vu.hh>
#include <cstdint>
#include <ee/ee_interpreter.hh>
#include <functional>
#include <memory>
#include <ee/ee_jit.hh>
#include <reg.hh>
#include <fstream>
#include <elf.h>
#include <cstring>

struct COP0 {
    union {
        uint32_t regs[32]; // Array access for indexed operations

        struct {
            uint32_t index;      // $0
            uint32_t random;     // $1
            uint32_t entryLo0;   // $2
            uint32_t entryLo1;   // $3
            uint32_t context;    // $4
            uint32_t pageMask;   // $5
            uint32_t wired;      // $6
            uint32_t _unused7;   // $7 (unused/reserved)
            uint32_t badVAddr;   // $8
            uint32_t count;      // $9
            uint32_t entryHi;    // $10
            uint32_t compare;    // $11
            
            // Nested union for Status register (allows bit-level access)
            union {
                uint32_t status; // $12 (full register access)

                struct {
                    uint32_t IE : 1;       // Bit 0: Interrupt Enable flag
                    uint32_t EXL : 1;      // Bit 1: Exception Level
                    uint32_t ERL : 1;      // Bit 2: Error Level
                    uint32_t KSU : 2;      // Bits 3-4: Kernel/User/Supervisor mode
                    uint32_t unused0 : 3;  // Bits 5-7: Reserved
                    uint32_t IM : 8;       // Bits 8-15: Interrupt Mask
                    uint32_t EIE : 1;      // Bit 16: Enable Interrupt Enable
                    uint32_t EDI : 1;      // Bit 17: Enable Dispatch Interrupt
                    uint32_t CH : 1;       // Bit 18: Cache Hit flag
                    uint32_t unused1 : 3;  // Bits 19-21: Reserved
                    uint32_t BEV : 1;      // Bit 22: Bootstrap Exception Vector
                    uint32_t DEV : 1;      // Bit 23: Debug Exception Vector
                    uint32_t unused2 : 2;  // Bits 24-25: Reserved
                    uint32_t FR : 1;       // Bit 26: FPU 64-bit mode
                    uint32_t unused3 : 1;  // Bit 27: Reserved
                    uint32_t CU : 4;       // Bits 28-31: Coprocessor Usable flags
                } Status; // Bit-level access
            };

            uint32_t cause;      // $13
            uint32_t epc;        // $14
            uint32_t PRId;       // $15
            uint32_t config;     // $16
            uint32_t _unused17;  // $17 (unused/reserved)
            uint32_t _unused18;  // $18 (unused/reserved)
            uint32_t _unused19;  // $19 (unused/reserved)
            uint32_t _unused20;  // $20 (unused/reserved)
            uint32_t _unused21;  // $21 (unused/reserved)
            uint32_t _unused22;  // $22 (unused/reserved)
            uint32_t badPAddr;   // $23
            uint32_t debug;      // $24
            uint32_t perf;       // $25
            uint32_t _unused26;  // $26 (unused/reserved)
            uint32_t _unused27;  // $27 (unused/reserved)
            uint32_t tagLo;      // $28
            uint32_t tagHi;      // $29
            uint32_t errorEPC;   // $30
            uint32_t _unused31;  // $31 (unused/reserved)
        };
    };

    // Operator[] to allow array-like access
    uint32_t& operator[](std::size_t index) {
        return regs[index];
    }

    const uint32_t& operator[](std::size_t index) const {
        return regs[index];
    }
};

class EE : public CPU
{
  public:
    EE(Bus *bus_, EmulationMode mode = EmulationMode::Interpreter);
    ~EE();

    std::function<void()> step_;
    std::function<void(Breakpoint*)> run_;
    void run(Breakpoint *breakpoints) override;
    void step() override;
    void reset();

    std::shared_ptr<const std::unordered_map<uint32_t, CompiledBlock>> get_block_cache() const override {
      // Return a shared pointer to the block cache if JIT is available
      return jit ? jit->get_block_cache() : nullptr;
    }

    std::uint32_t fetch_opcode() override;
    std::uint32_t fetch_opcode(std::uint32_t pc_);
    void parse_opcode(std::uint32_t opcode) override;
    void unknown_opcode(std::uint32_t opcode);
    void set_backend(EmulationMode mode);

    // 104 MIPS III/IV Instructions
    // 111 EE-Specific (SIMD-Like) Instructions
    std::function<void(EE *, std::uint32_t)> opcodes[104 + 111];

    uint128_t registers[32];
    COP0 cop0;
    fpu_reg_t fpr[32];

    uint128_t lo;
    uint128_t hi;

    std::uint64_t cycles = 0;

    VU vu0;
    VU vu1;

    std::uint32_t pc = 0xBFC00000;
    std::uint32_t next_pc;
    bool branching = false;
    bool likely_branch = false;
    std::uint32_t branch_dest;

    bool sideload_elf = false;
    uint32_t elf_entry_point = 0;
    std::string elf_path;

    void load_elf(const std::string& elf_path);
    void set_elf_state(bool state);

  private:
    std::unique_ptr<EEJIT> jit;
};
