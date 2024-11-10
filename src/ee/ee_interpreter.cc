#include <ee/ee_interpreter.hh>
#include <neo2.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

void ee_interpreter_setup(EE *ee)
{
	Logger::info("Populating opcode table...");
    ee->opcodes[0x10] = ee_interp_mfc0;
}

void ee_step_interpreter(EE *ee)
{
    std::uint32_t opcode = ee->fetch_opcode();
    ee->pc = ee->next_pc;
    ee->next_pc += 4;
    ee->parse_opcode(opcode);
}

void ee_interp_mfc0(EE *ee, std::uint32_t opcode)
{
    printf("Hello\n");
}
