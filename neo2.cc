#include <neo2.hh>

Neo2::Neo2() : bus(BusMode::SoftwareFastMem), ee(&bus, EmulationMode::Interpreter)
{
    Logger::raw("Neo2 - Initialized Neo2 system with EE and Bus.");
}

Neo2::~Neo2()
{
    Logger::raw("Neo2 - Shutting down Neo2 system.");
}

void Neo2::run_ee()
{
    Logger::raw("Neo2 - Starting EE emulation.");
    ee.run();
}
