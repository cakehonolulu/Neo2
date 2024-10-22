#include <neo2.hh>

Neo2::Neo2() : bus(BusMode::SoftwareFastMem), ee(&bus, EmulationMode::Interpreter)
{
	Logger::raw("  _   _           ____  \n"
				" | \\ | | ___  ___|___ \\ \n"
				" |  \\| |/ _ \\/ _ \\ __) |\n"
				" | |\\  |  __/ (_) / __/ \n"
				" |_| \\_|\\___|\\___/_____|   -   Simple Sony PlayStation 2 Emulator\n");
}

Neo2::~Neo2()
{
    Logger::raw("Neo2 - Shutting down Neo2 system.");
}

void Neo2::run_ee()
{
    ee.run();
}
