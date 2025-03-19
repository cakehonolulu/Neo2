#include "cpu/breakpoint.hh"
#include <constants.hh>
#include <iop/iop_jit.hh>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/raw_ostream.h>
#include <log/log.hh>
#include <neo2.hh>
#include <sstream>
#include <thread>

extern "C" EXPORT uint32_t iop_read32(IOP *core, uint32_t addr)
{
    return core->bus->read32(addr);
}

extern "C" EXPORT void iop_write8(IOP *core, uint32_t addr, uint32_t value)
{
    core->bus->write8(addr, value);
}

extern "C" EXPORT void iop_write32(IOP *core, uint32_t addr, uint32_t value)
{
    core->bus->write32(addr, value);
}

void IOPJIT::setup_iop_jit_primitives(std::unique_ptr<llvm::Module> &new_module)
{
    iop_read32_type = llvm::FunctionType::get(
        builder->getInt32Ty(), {llvm::PointerType::getUnqual(builder->getInt8Ty()), builder->getInt32Ty()}, false);

    iop_read32 =
        llvm::Function::Create(iop_read32_type, llvm::Function::ExternalLinkage, "iop_read32", new_module.get());

    iop_write8_type = llvm::FunctionType::get(
        llvm::Type::getVoidTy(*context), {builder->getInt64Ty(), builder->getInt32Ty(), builder->getInt8Ty()}, false);

    iop_write8 =
        llvm::Function::Create(iop_write8_type, llvm::Function::ExternalLinkage, "iop_write8", new_module.get());

    iop_write32_type = llvm::FunctionType::get(
        llvm::Type::getVoidTy(*context), {builder->getInt64Ty(), builder->getInt32Ty(), builder->getInt32Ty()}, false);

    iop_write32 =
        llvm::Function::Create(iop_write32_type, llvm::Function::ExternalLinkage, "iop_write32", new_module.get());
}
