#pragma once
#include <reg.hh>
#include <cstdint>
#include <string>
#include <unordered_map>

class GS {
public:
    // Constants for register addresses
    static constexpr uint32_t BASE_ADDRESS = 0x12000000;
    static constexpr uint32_t CSR_ADDRESS = 0x12001000;
    static constexpr uint32_t IMR_ADDRESS = 0x12001010;
    static constexpr uint32_t BUSDIR_ADDRESS = 0x12001040;
    static constexpr uint32_t SIGLBLID_ADDRESS = 0x12001080;

    static constexpr const char* REGISTER_NAMES[19] = {
        "PMODE", "SMODE1", "SMODE2", "SRFSH", "SYNCH1", "SYNCH2", "SYNCV", 
        "DISPFB1", "DISPLAY1", "DISPFB2", "DISPLAY2", "EXTBUF", "EXTDATA", 
        "EXTWRITE", "BGCOLOR", "GS_CSR", "GS_IMR", "BUSDIR", "SIGLBLID"
    };

    static constexpr const char* INTERNAL_REGISTER_NAMES[57] = {
        "PRIM", "RGBAQ", "ST", "UV", "XYZF2", "XYZ2", "TEX0_1", "TEX0_2", 
        "CLAMP_1", "CLAMP_2", "FOG", "", "XYZF3", "XYZ3", "TEX1_1", "TEX1_2", 
        "TEX2_1", "TEX2_2", "XYOFFSET_1", "XYOFFSET_2", "PRMODECONT", "PRMODE", 
        "TEXCLUT", "", "", "", "", "", "TEXA", "", "FOGCOL", "", "TEXFLUSH", 
        "SCISSOR_1", "SCISSOR_2", "ALPHA_1", "ALPHA_2", "DIMX", "DTHE", "COLCLAMP", 
        "TEST_1", "TEST_2", "PABE", "FBA_1", "FBA_2", "FRAME_1", "FRAME_2", 
        "ZBUF_1", "ZBUF_2", "BITBLTBUF", "TRXPOS", "TRXREG", "TRXDIR", "HWREG", 
        "SIGNAL", "FINISH", "LABEL"
    };

    static constexpr std::size_t VRAM_SIZE = 4 * 1024 * 1024; // 4MB VRAM size

    GS();
    ~GS();

    // Methods to handle register reads and writes
    uint64_t read(uint32_t address);
    void write(uint32_t address, uint64_t value);

    void simul_vblank();

    // Method to handle incoming GIF data
    void write_gif_data(uint64_t data);
    void write_internal_reg(uint8_t reg, uint64_t data);
    void write_packed_gif_data(uint8_t reg, uint128_t data);

    void set_bitbltbuf(uint64_t value);
    void set_trxpos(uint64_t value);
    void set_trxreg(uint64_t value);
    void set_trxdir(uint64_t value);
    void transfer_vram();

    uint32_t* vram; // VRAM pointer

    uint64_t gs_privileged_registers[19]; // 19 privileged registers
    uint64_t gs_registers[55]; // 55 general registers

    void write_hwreg(uint64_t data);
private:
    uint64_t bitbltbuf = 0;
    uint64_t trxpos = 0;
    uint64_t trxreg = 0;
    uint64_t trxdir = 0;
    uint64_t hwreg = 0;

    // 50h BLITBLTBUF
    uint32_t source_base_pointer = 0;
    uint32_t source_buffer_width = 0;
    uint32_t source_format = 0;
    uint32_t destination_base_pointer = 0;
    uint32_t destination_buffer_width = 0;
    uint32_t destination_format = 0;

    // 51h TRXPOS
    uint32_t source_rectangle_x = 0;
    uint32_t source_rectangle_y = 0;
    uint32_t destination_rectangle_x = 0;
    uint32_t destination_rectangle_y = 0;
    uint32_t transmission_order = 0;

    // 52h TRXREG
    uint32_t transmission_area_pixel_width = 0;
    uint32_t transmission_area_pixel_height = 0;

    // 53h TRXDIR
    uint32_t transmission_direction = 0;

    // GIF->VRAM aux.
    uint32_t source_x = 0;
    uint32_t source_y = 0;
    uint32_t destination_x = 0;
    uint32_t destination_y = 0;
};
