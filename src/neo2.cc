#include <neo2.hh>
#include <sstream>

std::string colorCode(int r, int g, int b) {
    return "\033[38;2;" + std::to_string(r) + ";" + std::to_string(g) + ";" + std::to_string(b) + "m";
}

std::string backgroundColorCode(int r, int g, int b) {
    return "\033[48;2;" + std::to_string(r) + ";" + std::to_string(g) + ";" + std::to_string(b) + "m";
}

std::string resetColor() {
    return "\033[0m";
}

Neo2::Neo2() : bus(BusMode::SoftwareFastMem), ee(&bus, EmulationMode::Interpreter)
{
    std::string banner =
        "  _   _           ____  \n"
        " | \\ | | ___  ___|___ \\ \n"
        " |  \\| |/ _ \\/ _ \\ __) |\n"
        " | |\\  |  __/ (_) / __/ \n"
        " |_| \\_|\\___|\\___/_____|  -  Simple Sony PlayStation 2 Emulator\n";

    int r_start = 48;
    int g_start = 78;
    int b_start = 151;

    int r_end = 0;
    int g_end = 173;
    int b_end = 204;

    int lines = 5;

    std::istringstream bannerStream(banner);
    std::string line;
    int line_index = 0;

    while (std::getline(bannerStream, line)) {
        float ratio = (float)line_index / (lines - 1);
        int r = r_start + static_cast<int>((r_end - r_start) * ratio);
        int g = g_start + static_cast<int>((g_end - g_start) * ratio);
        int b = b_start + static_cast<int>((b_end - b_start) * ratio);

        Logger::raw("\033[1m" + backgroundColorCode(r, g, b) + colorCode(255, 255, 255) + line + resetColor() + "\n");
        line_index++;
    }
}

Neo2::~Neo2()
{
    Logger::raw("Neo2 - Shutting down Neo2 system.");
}

/*void Neo2::run_ee()
{
    ee.run();
}*/
