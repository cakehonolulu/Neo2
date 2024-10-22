#include <bus/bus.hh>
#include <ee/ee.hh>
#include <iostream>
#include <neo2.hh>
#include <string>
#include <vector>
#include <log/log.hh>
#include <log/log_term.hh>

int main(int argc, char **argv)
{
    const std::string bios_arg = "-bios";
    std::string bios_file;

    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " -bios <bios_file>\n";
        return 1;
    }
    else
    {
        for (int i = 1; i < argc; i++)
        {
            if (bios_arg.compare(argv[i]) == 0)
            {
                if (argv[i + 1] != nullptr)
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

    std::shared_ptr<LogBackend> terminal_backend = std::make_shared<TerminalLogBackend>();
    Logger::add_backend(terminal_backend);

	Neo2 neo2;
	neo2.run_ee();

    return 0;
}
