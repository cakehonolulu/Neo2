#include <iop/iop_interpreter.hh>
#include <neo2.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

void iop_interpreter_setup(IOP *iop)
{
	Logger::info("Populating opcode table...");
}

void iop_step_interpreter(IOP *iop)
{
    std::uint32_t opcode = iop->fetch_opcode();
    iop->old_pc = iop->pc;
    iop->pc = iop->next_pc;
    iop->next_pc += 4;
    iop->parse_opcode(opcode);
}
