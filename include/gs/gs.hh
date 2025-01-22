#pragma once
#include <cstdint>

class GS {
public:
    // Constants for register addresses
    static constexpr uint32_t PMODE = 0x12000000;
    static constexpr uint32_t SMODE1 = 0x12000010;
    static constexpr uint32_t SMODE2 = 0x12000020;
    static constexpr uint32_t SRFSH = 0x12000030;
    static constexpr uint32_t SYNCH1 = 0x12000040;
    static constexpr uint32_t SYNCH2 = 0x12000050;
    static constexpr uint32_t SYNCV = 0x12000060;
    static constexpr uint32_t DISPFB1 = 0x12000070;
    static constexpr uint32_t DISPLAY1 = 0x12000080;
    static constexpr uint32_t DISPFB2 = 0x12000090;
    static constexpr uint32_t DISPLAY2 = 0x120000A0;
    static constexpr uint32_t EXTBUF = 0x120000B0;
    static constexpr uint32_t EXTDATA = 0x120000C0;
    static constexpr uint32_t EXTWRITE = 0x120000D0;
    static constexpr uint32_t BGCOLOR = 0x120000E0;
    static constexpr uint32_t GS_CSR = 0x12001000;
    static constexpr uint32_t GS_IMR = 0x12001010;
    static constexpr uint32_t BUSDIR = 0x12001040;
    static constexpr uint32_t SIGLBLID = 0x12001080;

    GS();

    // Methods to handle register reads and writes
    uint64_t read(uint32_t address);
    void write(uint32_t address, uint64_t value);

private:
    uint64_t gs_registers[19]; // 19 registers in total
};
