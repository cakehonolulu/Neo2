#include <neo2.hh>
#include <sstream>
#include <typeinfo>
#include <log/log_imgui.hh>
#include <log/log_term.hh>

std::string colorCode(int r, int g, int b) {
    return "\033[38;2;" + std::to_string(r) + ";" + std::to_string(g) + ";" + std::to_string(b) + "m";
}

std::string backgroundColorCode(int r, int g, int b) {
    return "\033[48;2;" + std::to_string(r) + ";" + std::to_string(g) + ";" + std::to_string(b) + "m";
}

std::string resetColor() {
    return "\033[0m";
}

Neo2::Neo2(std::shared_ptr<LogBackend> logger)
    : bus(BusMode::SoftwareFastMem), ee(&bus, EmulationMode::Interpreter), iop(&bus, EmulationMode::Interpreter) {
    if (logger) {
        Logger::add_backend(logger);
    }

    // Define the banner
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

    // Detect backend type
    bool is_terminal = false;
    ImGuiLogBackend* imgui_backend = nullptr;

    if (!Logger::get_backends().empty()) {
        // Check if the logger is an instance of ImGuiLogBackend
        imgui_backend = dynamic_cast<ImGuiLogBackend*>(Logger::get_backends().front().get());
        is_terminal = dynamic_cast<TerminalLogBackend*>(Logger::get_backends().front().get()) != nullptr;
    }

    // Print each line of the banner based on frontend type
    while (std::getline(bannerStream, line)) {
        float ratio = (float)line_index / (lines - 1);
        int r = r_start + static_cast<int>((r_end - r_start) * ratio);
        int g = g_start + static_cast<int>((g_end - g_start) * ratio);
        int b = b_start + static_cast<int>((b_end - b_start) * ratio);

        if (is_terminal) {
            Logger::raw("\033[1m" + backgroundColorCode(r, g, b) + colorCode(255, 255, 255) + line + resetColor() + "\n");
        } else if (imgui_backend) {
            std::ostringstream oss;
            oss << "[fg_color(255,255,255),bg_color(" << r << "," << g << "," << b << ")]" << line;
            imgui_backend->raw_special(oss.str(), LogLevel::Raw);
        }

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
