#pragma once
#include <cstdint>

class GIF {
public:
    // Constants for register addresses
    static constexpr uint32_t GIF_CTRL = 0x10003000;
    static constexpr uint32_t GIF_MODE = 0x10003010;
    static constexpr uint32_t GIF_STAT = 0x10003020;
    static constexpr uint32_t GIF_TAG0 = 0x10003040;
    static constexpr uint32_t GIF_TAG1 = 0x10003050;
    static constexpr uint32_t GIF_TAG2 = 0x10003060;
    static constexpr uint32_t GIF_TAG3 = 0x10003070;
    static constexpr uint32_t GIF_CNT = 0x10003080;
    static constexpr uint32_t GIF_P3CNT = 0x10003090;
    static constexpr uint32_t GIF_P3TAG = 0x100030A0;
    static constexpr uint32_t GIF_FIFO = 0x10006000;

    GIF();

    // Methods to handle register reads and writes
    uint32_t read(uint32_t address);
    void write(uint32_t address, uint32_t value);

private:
    uint32_t gif_ctrl;
    uint32_t gif_mode;
    uint32_t gif_stat;
    uint32_t gif_tag[4];
    uint32_t gif_cnt;
    uint32_t gif_p3cnt;
    uint32_t gif_p3tag;
    uint32_t gif_fifo[4]; // GIF_FIFO size is 16 bytes (4 words)
};
