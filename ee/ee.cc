#include <cstring>
#include <ee/ee.hh>
#include <iostream>
#include <neo2.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

EE::EE(Bus *bus_, EmulationMode mode)
{
    bus = bus_;

    switch (mode)
    {
    case EmulationMode::Interpreter:
        std::cout << CYAN << "[EE] Running in Interpreter mode..." << RESET "\n";
        ee_step = std::bind(&ee_step_interpreter, this);
        break;
    case EmulationMode::CachedInterpreter:
        std::cerr << BOLDRED << "[EE] Cached interpreter mode is unavailable" << RESET "\n";
        exit(1);
        break;
    default:
        std::cerr << BOLDRED << "[EE] Invalid emulation mode" << RESET "\n";
        exit(1);
        break;
    }

    std::memset(registers, 0, sizeof(registers));
    pc = 0xBFC00000;
    next_pc = pc + 4;
}

EE::~EE()
{
}

void EE::run()
{
    while (true)
    {
        ee_step();
    }
}
