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

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

static const char* ee_get_syscall(int n) {
    switch (n) {
        case 0x01: return "void ResetEE(int reset_flag)";
        case 0x02: return "void SetGsCrt(bool interlaced, int display_mode, bool frame)";
        case 0x04: return "void Exit(int status)";
        case 0x05: return "void _ExceptionEpilogue()";
        case 0x06: return "void LoadExecPS2(const char* filename, int argc, char** argv)";
        case 0x07: return "void ExecPS2(void* entry, void* gp, int argc, char** argv)";
        case 0x10: return "int AddIntcHandler(int int_cause, int (*handler)(int), int next, void* arg, int flag)";
        case 0x11: return "int RemoveIntcHandler(int int_cause, int handler_id)";
        case 0x12: return "int AddDmacHandler(int dma_cause, int (*handler)(int), int next, void* arg, int flag)";
        case 0x13: return "int RemoveDmacHandler(int dma_cause, int handler_id)";
        case 0x14: return "bool _EnableIntc(int cause_bit)";
        case 0x15: return "bool _DisableIntc(int cause_bit)";
        case 0x16: return "bool _EnableDmac(int cause_bit)";
        case 0x17: return "bool _DisableDmac(int cause_bit)";
        case 0x20: return "int CreateThread(ThreadParam* t)";
        case 0x21: return "void DeleteThread(int thread_id)";
        case 0x22: return "void StartThread(int thread_id, void* arg)";
        case 0x23: return "void ExitThread()";
        case 0x24: return "void ExitDeleteThread()";
        case 0x25: return "void TerminateThread(int thread_id)";
        case 0x26: return "void iTerminateThread(int thread_id)";
        case 0x29: return "int ChangeThreadPriority(int thread_id, int priority)";
        case 0x2A: return "int iChangeThreadPriority(int thread_id, int priority)";
        case 0x2B: return "void RotateThreadReadyQueue(int priority)";
        case 0x2C: return "int _iRotateThreadReadyQueue(int priority)";
        case 0x2D: return "void ReleaseWaitThread(int thread_id)";
        case 0x2E: return "int iReleaseWaitThread(int thread_id)";
        case 0x2F: return "int GetThreadId()";
        case 0x30: return "int ReferThreadStatus(int thread_id, ThreadParam* status)";
        case 0x31: return "int iReferThreadStatus(int thread_id, ThreadParam* status)";
        case 0x32: return "void SleepThread()";
        case 0x33: return "void WakeupThread(int thread_id)";
        case 0x34: return "int iWakeupThread(int thread_id)";
        case 0x35: return "int CancelWakeupThread(int thread_id)";
        case 0x36: return "int iCancelWakeupThread(int thread_id)";
        case 0x37: return "int SuspendThread(int thread_id)";
        case 0x38: return "int iSuspendThread(int thread_id)";
        case 0x39: return "void ResumeThread(int thread_id)";
        case 0x3A: return "int iResumeThread(int thread_id)";
        case 0x3B: return "void JoinThread()";
        case 0x3C: return "void* InitMainThread(uint32 gp, void* stack, int stack_size, char* args, int root)";
        case 0x3D: return "void* InitHeap(void* heap, int heap_size)";
        case 0x3E: return "void* EndOfHeap()";
        case 0x40: return "int CreateSema(SemaParam* s)";
        case 0x41: return "int DeleteSema(int sema_id)";
        case 0x42: return "int SignalSema(int sema_id)";
        case 0x43: return "int iSignalSema(int sema_id)";
        case 0x44: return "void WaitSema(int sema_id)";
        case 0x45: return "int PollSema(int sema_id)";
        case 0x46: return "int iPollSema(int sema_id)";
        case 0x64: return "void FlushCache(int mode)";
        case 0x70: return "uint64_t GsGetIMR()";
        case 0x71: return "void GsPutIMR(uint64_t value)";
        case 0x73: return "void SetVSyncFlag(int* vsync_occurred, u64* csr_stat_on_vsync)";
        case 0x74: return "void SetSyscall(int index, int address)";
        case 0x76: return "int SifDmaStat(unsigned int dma_id)";
        case 0x77: return "unsigned int SifSetDma(SifDmaTransfer* trans, int len)";
        case 0x78: return "void SifSetDChain()";
        case 0x7B: return "void ExecOSD(int argc, char** argv)";
        case 0x7D: return "void PSMode()";
        case 0x7E: return "int MachineType()";
        case 0x7F: return "int GetMemorySize()";
    }

    return "<unknown>";
}

extern "C" EXPORT void ee_print_syscall(EE* core, uint32_t current_pc, uint32_t syscall_nr) {
    Logger::error(ee_get_syscall(syscall_nr));
}

void update_address_mapping(EE* core, uint32_t entry_hi, uint32_t page_mask, uint32_t entry_lo0, uint32_t entry_lo1, bool global) {
    // Update the address spaces with the new TLB entry
    uint32_t vpn2_base = entry_hi & ~(page_mask);
    for (uint32_t i = 0; i < (page_mask >> 12) + 1; ++i) {
        uint32_t vpn2 = vpn2_base + (i << 12);

        // Map entry_lo0
        uintptr_t pfn0 = entry_lo0 << 12;
        core->bus->address_space_r[vpn2 >> 12] = (uintptr_t)&core->bus->ram[pfn0];
        core->bus->address_space_w[vpn2 >> 12] = (uintptr_t)&core->bus->ram[pfn0];

        // Map entry_lo1
        uintptr_t pfn1 = entry_lo1 << 12;
        core->bus->address_space_r[(vpn2 + (1 << 12)) >> 12] = (uintptr_t)&core->bus->ram[pfn1];
        core->bus->address_space_w[(vpn2 + (1 << 12)) >> 12] = (uintptr_t)&core->bus->ram[pfn1];
    }
}

extern "C" EXPORT void ee_update_address_mapping(EE* core, uint32_t entry_hi, uint32_t page_mask, uint32_t entry_lo0, uint32_t entry_lo1, bool global) {
    update_address_mapping(core, entry_hi, page_mask, entry_lo0, entry_lo1, global);
}

extern "C" EXPORT void ee_write32_dbg(EE* core, uint32_t addr, uint32_t value, uint32_t pc) {
    core->bus->write32_dbg(addr, value, pc);
}

extern "C" EXPORT void ee_write8(EE* core, uint32_t addr, uint8_t value) {
    core->bus->write8(addr, value);
}

extern "C" EXPORT void ee_write16(EE* core, uint32_t addr, uint16_t value) {
    core->bus->write16(addr, value);
}

extern "C" EXPORT void ee_write32(EE* core, uint32_t addr, uint32_t value) {
    core->bus->write32(addr, value);
}

extern "C" EXPORT void ee_write64(EE* core, uint32_t addr, uint64_t value) {
    core->bus->write64(addr, value);
}

extern "C" EXPORT void ee_write128(EE* core, uint32_t addr, uint128_t value) {
    core->bus->write128(addr, value);
}

extern "C" EXPORT uint8_t ee_read8(EE* core, uint32_t addr) {
    return core->bus->read8(addr);
}

extern "C" EXPORT uint16_t ee_read16(EE* core, uint32_t addr) {
    return core->bus->read16(addr);
}

extern "C" EXPORT uint32_t ee_read32(EE* core, uint32_t addr) {
    return core->bus->read32(addr);
}

extern "C" EXPORT uint64_t ee_read64(EE* core, uint32_t addr) {
    return core->bus->read64(addr);
}

extern "C" EXPORT uint128_t ee_read128(EE* core, uint32_t addr) {
    return core->bus->read128(addr);
}

extern "C" EXPORT void ee_tlb_write(EE* core, uint32_t index, uint32_t PageMask, uint32_t EntryHi, uint32_t EntryLo0, uint32_t EntryLo1) {
    core->bus->tlb.write_entry(index, PageMask, EntryHi, EntryLo0, EntryLo1);
}

extern "C" EXPORT void ee_load_elf(EE* core) {
    core->load_elf(core->elf_path);
}

void EEJIT::setup_ee_jit_primitives(std::unique_ptr<llvm::Module>& new_module) {
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
        "ee_write8", new_module.get()
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
        "ee_write16", new_module.get()
    );

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
        "ee_write32", new_module.get()
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
        new_module.get()
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
        new_module.get()
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
        new_module.get()
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
        new_module.get()
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
        new_module.get()
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
        new_module.get()
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
        new_module.get()
    );

    ee_tlb_write_type = llvm::FunctionType::get(
        llvm::Type::getVoidTy(*context),
        {llvm::PointerType::getUnqual(builder->getInt8Ty()), // EE*
        builder->getInt32Ty(), // Index
        builder->getInt32Ty(), // PageMask
        builder->getInt32Ty(), // EntryHi
        builder->getInt32Ty(), // EntryLo0
        builder->getInt32Ty(), // EntryLo1
        },
        false
    );

    ee_tlb_write = llvm::Function::Create(
        ee_tlb_write_type,
        llvm::Function::ExternalLinkage,
        "ee_tlb_write",
        new_module.get()
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
        "ee_write32_dbg", new_module.get()
    );

    ee_update_address_mapping_type = llvm::FunctionType::get(
        llvm::Type::getVoidTy(*context),
        {
            builder->getInt64Ty(), // EE* core
            builder->getInt32Ty(),                              // uint32_t entry_hi
            builder->getInt32Ty(),                              // uint32_t page_mask
            builder->getInt32Ty(),                              // uint32_t entry_lo0
            builder->getInt32Ty(),                              // uint32_t entry_lo1
            builder->getInt1Ty()                                // bool global
        },
        false
    );

    // Create the function
    ee_update_address_mapping = llvm::Function::Create(
        ee_update_address_mapping_type,
        llvm::Function::ExternalLinkage,
        "ee_update_address_mapping",
        new_module.get()
    );

    ee_load_elf_type = llvm::FunctionType::get(
        builder->getInt8Ty(),
        {llvm::PointerType::getUnqual(builder->getInt8Ty())},
        false
    );

    ee_load_elf = llvm::Function::Create(
        ee_load_elf_type,
        llvm::Function::ExternalLinkage,
        "ee_load_elf",
        new_module.get()
    );

    ee_print_syscall_type = llvm::FunctionType::get(
        llvm::Type::getVoidTy(*context),
        {llvm::PointerType::getUnqual(builder->getInt8Ty()), // EE*
        builder->getInt32Ty(), // current_pc
        builder->getInt32Ty(), // syscall_nr
        },
        false
    );

    ee_print_syscall = llvm::Function::Create(
        ee_print_syscall_type,
        llvm::Function::ExternalLinkage,
        "ee_print_syscall",
        new_module.get()
    );
}
