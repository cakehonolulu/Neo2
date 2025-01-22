#include <gif/gif.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

GIF::GIF()
    : gif_ctrl(0),
      gif_mode(0),
      gif_stat(0),
      gif_cnt(0),
      gif_p3cnt(0),
      gif_p3tag(0) {
    Logger::set_subsystem("GIF");
    for (int i = 0; i < 4; ++i) {
        gif_tag[i] = 0;
        gif_fifo[i] = 0;
    }
}

uint32_t GIF::read(uint32_t address) {
    std::string reg_name;
    switch (address) {
        case GIF_STAT:
            reg_name = "GIF_STAT";
            break;
        case GIF_TAG0:
            reg_name = "GIF_TAG0";
            break;
        case GIF_TAG1:
            reg_name = "GIF_TAG1";
            break;
        case GIF_TAG2:
            reg_name = "GIF_TAG2";
            break;
        case GIF_TAG3:
            reg_name = "GIF_TAG3";
            break;
        case GIF_CNT:
            reg_name = "GIF_CNT";
            break;
        case GIF_P3CNT:
            reg_name = "GIF_P3CNT";
            break;
        case GIF_P3TAG:
            reg_name = "GIF_P3TAG";
            break;
        case GIF_FIFO:
        case GIF_FIFO + 4:
        case GIF_FIFO + 8:
        case GIF_FIFO + 12:
            reg_name = format("GIF_FIFO+{}", (address - GIF_FIFO) / 4);
            break;
        default:
            Logger::error("Invalid GIF register read at address 0x" + format("{:08X}", address));
            return 0;
    }
    Logger::info("GIF register read from " + reg_name);
    return (reg_name.rfind("GIF_FIFO", 0) == 0) ? gif_fifo[(address - GIF_FIFO) / 4] : *(reinterpret_cast<uint32_t*>(reinterpret_cast<uint8_t*>(this) + (address - GIF_CTRL)));
}

void GIF::write(uint32_t address, uint32_t value) {
    std::string reg_name;
    switch (address) {
        case GIF_CTRL:
            reg_name = "GIF_CTRL";
            gif_ctrl = value;
            if (value & 0x1) {
                // Reset GIF
                gif_stat = 0;
                for (int i = 0; i < 4; ++i) {
                    gif_tag[i] = 0;
                }
                gif_cnt = 0;
                gif_p3cnt = 0;
                gif_p3tag = 0;
            }
            break;
        case GIF_MODE:
            reg_name = "GIF_MODE";
            gif_mode = value;
            break;
        case GIF_FIFO:
        case GIF_FIFO + 4:
        case GIF_FIFO + 8:
        case GIF_FIFO + 12:
            reg_name = format("GIF_FIFO+{}", (address - GIF_FIFO) / 4);
            gif_fifo[(address - GIF_FIFO) / 4] = value;
            break;
        default:
            Logger::error("Invalid GIF register write at address 0x" + format("{:08X}", address));
            return;
    }
    Logger::info("GIF register write to " + reg_name + " with value 0x" + format("{:08X}", value));
}
