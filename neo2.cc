#include <bus/bus.hh>
#include <ee/ee.hh>
#include <iostream>
#include <neo2.hh>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
    const std::string bios_arg = "-bios";
    std::string bios_file;

    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <bios_file>\n";
        return 1;
    }
    else
    {
        for (int i = 1; i < argc; i++)
        {
            if (bios_arg.compare(argv[i]) == 0)
            {
                if (argv[i + 1] != NULL)
                {
                    bios_file = argv[i + 1];
                    i++;
                }
                else
                {
                    std::cerr << "No bios file provided!\n";
                    return 1;
                }
            }
        }
    }

    std::cout << MAGENTA << "Neo2 - A simple, PlayStation 2 Emulator" << RESET "\n";

    // Initialize the Bus Interconnector
    Bus bus;
    bus.load_bios(bios_file);

    // Initialize the EE
    EE ee(&bus, EmulationMode::Interpreter);
    ee.run();

    return 0;
}
