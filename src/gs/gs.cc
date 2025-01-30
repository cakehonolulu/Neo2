#include "neo2.hh"
#include <cstdio>
#include <gs/gs.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

#include <cstring>

GS::GS() {
    for (int i = 0; i < 19; ++i) {
        gs_privileged_registers[i] = 0;
    }
    vram = new uint32_t[VRAM_SIZE]();
    Logger::set_subsystem("GS");
}

GS::~GS() {
    delete[] vram;
}

uint64_t GS::read(uint32_t address) {
    int index = -1;
    if (address < CSR_ADDRESS) {
        index = (address - BASE_ADDRESS) / 0x10;
    } else if (address == CSR_ADDRESS) {
        index = 15;
    } else if (address == IMR_ADDRESS) {
        index = 16;
    } else if (address == BUSDIR_ADDRESS) {
        index = 17;
    } else if (address == SIGLBLID_ADDRESS) {
        index = 18;
    } else {
        Logger::error("Invalid GS register read at address 0x" + format("{:08X}", address));
        return Neo2::exit(1, Neo2::Subsystem::GS);
    }

    Logger::info("GS register read from " + std::string(REGISTER_NAMES[index]));
    return gs_privileged_registers[index];
}

void GS::write(uint32_t address, uint64_t value) {
    int index = -1;
    if (address < CSR_ADDRESS) {
        index = (address - BASE_ADDRESS) / 0x10;
    } else if (address == CSR_ADDRESS) {
        index = 15;
    } else if (address == IMR_ADDRESS) {
        index = 16;
    } else if (address == BUSDIR_ADDRESS) {
        index = 17;
    } else if (address == SIGLBLID_ADDRESS) {
        index = 18;
    } else {
        Logger::error("Invalid GS register write at address 0x" + format("{:08X}", address));
        Neo2::exit(1, Neo2::Subsystem::GS);
    }

    Logger::info("GS register write to " + std::string(REGISTER_NAMES[index]) + " with value 0x" + format("{:08X}", value));
    gs_privileged_registers[index] = value;
}

void GS::simul_vblank() {
    // Simulate GS operation complete by toggling bit 3
    gs_privileged_registers[15] |= 0x8; // Set bit 3
}

void GS::write_internal_reg(uint8_t reg, uint64_t data) {
    switch (reg) {
        case 0x00: // PRIM
            gs_registers[0x00] = data;
            break;
        case 0x01: // RGBAQ
            gs_registers[0x01] = data;
            break;
        case 0x02: // ST
            gs_registers[0x02] = data;
            break;
        case 0x03: // UV
            gs_registers[0x03] = data;
            break;
        case 0x04: // XYZF2
            gs_registers[0x04] = data;
            break;
        case 0x05: // XYZ2
            gs_registers[0x05] = data;
            break;
        case 0x06: // TEX0_1
            gs_registers[0x06] = data;
            break;
        case 0x07: // TEX0_2
            gs_registers[0x07] = data;
            break;
        case 0x08: // CLAMP_1
            gs_registers[0x08] = data;
            break;
        case 0x09: // CLAMP_2
            gs_registers[0x09] = data;
            break;
        case 0x0A: // FOG
            gs_registers[0x0A] = data;
            break;
        case 0x0C: // XYZF3
            gs_registers[0x0C] = data;
            break;
        case 0x0D: // XYZ3
            gs_registers[0x0D] = data;
            break;
        case 0x14: // TEX1_1
            gs_registers[0x14] = data;
            break;
        case 0x15: // TEX1_2
            gs_registers[0x15] = data;
            break;
        case 0x16: // TEX2_1
            gs_registers[0x16] = data;
            break;
        case 0x17: // TEX2_2
            gs_registers[0x17] = data;
            break;
        case 0x18: // XYOFFSET_1
            gs_registers[0x18] = data;
            break;
        case 0x19: // XYOFFSET_2
            gs_registers[0x19] = data;
            break;
        case 0x1A: // PRMODECONT
            gs_registers[0x1A] = data;
            break;
        case 0x1B: // PRMODE
            gs_registers[0x1B] = data;
            break;
        case 0x1C: // TEXCLUT
            gs_registers[0x1C] = data;
            break;
        case 0x22: // SCANMSK
            gs_registers[0x22] = data;
            break;
        case 0x34: // MIPTBP1_1
            gs_registers[0x34] = data;
            break;
        case 0x35: // MIPTBP1_2
            gs_registers[0x35] = data;
            break;
        case 0x36: // MIPTBP2_1
            gs_registers[0x36] = data;
            break;
        case 0x37: // MIPTBP2_2
            gs_registers[0x37] = data;
            break;
        case 0x3B: // TEXA
            gs_registers[0x3B] = data;
            break;
        case 0x3D: // FOGCOL
            gs_registers[0x3D] = data;
            break;
        case 0x3F: // TEXFLUSH
            gs_registers[0x3F] = data;
            break;
        case 0x40: // SCISSOR_1
            gs_registers[0x40] = data;
            break;
        case 0x41: // SCISSOR_2
            gs_registers[0x41] = data;
            break;
        case 0x42: // ALPHA_1
            gs_registers[0x42] = data;
            break;
        case 0x43: // ALPHA_2
            gs_registers[0x43] = data;
            break;
        case 0x44: // DIMX
            gs_registers[0x44] = data;
            break;
        case 0x45: // DTHE
            gs_registers[0x45] = data;
            break;
        case 0x46: // COLCLAMP
            gs_registers[0x46] = data;
            break;
        case 0x47: // TEST_1
            gs_registers[0x47] = data;
            break;
        case 0x48: // TEST_2
            gs_registers[0x48] = data;
            break;
        case 0x49: // PABE
            gs_registers[0x49] = data;
            break;
        case 0x4A: // FBA_1
            gs_registers[0x4A] = data;
            break;
        case 0x4B: // FBA_2
            gs_registers[0x4B] = data;
            break;
        case 0x4C: // FRAME_1
            gs_registers[0x4C] = data;
            break;
        case 0x4D: // FRAME_2
            gs_registers[0x4D] = data;
            break;
        case 0x4E: // ZBUF_1
            gs_registers[0x4E] = data;
            break;
        case 0x4F: // ZBUF_2
            gs_registers[0x4F] = data;
            break;
        case 0x50: // BITBLTBUF
            set_bitbltbuf(data);
            break;
        case 0x51: // TRXPOS
            set_trxpos(data);
            break;
        case 0x52: // TRXREG
            set_trxreg(data);
            break;
        case 0x53: // TRXDIR
            set_trxdir(data);
            break;
        case 0x54: // HWREG
            write_hwreg(data);
            break;
        case 0x60: // SIGNAL
            gs_registers[0x60] = data;
            break;
        case 0x61: // FINISH
            gs_registers[0x61] = data;
            break;
        case 0x62: // LABEL
            gs_registers[0x62] = data;
            break;
        default:
            Logger::error("Invalid GS internal register write to 0x" + format("{:X}", reg));
            Neo2::exit(1, Neo2::Subsystem::GS);
            break;
    }
}

void GS::write_packed_gif_data(uint8_t reg, uint128_t data) {
    // Handle the incoming GIF data and write to appropriate GS registers
    switch (reg) {
        case 0x0: // PRIM
            gs_registers[0] = data.u64[0];
            Logger::info("PRIM register 0x" + format("{:016X}", data.u64[0]));
            break;
        case 0x1: // RGBAQ
            gs_registers[1] = data.u64[0];
            Logger::info("RGBAQ register 0x" + format("{:016X}", data.u64[0]));
            break;
        case 0x2: // ST
            gs_registers[2] = data.u64[0];
            Logger::info("ST register 0x" + format("{:016X}", data.u64[0]));
            break;
        case 0x3: // UV
            gs_registers[3] = data.u64[0];
            Logger::info("UV register 0x" + format("{:016X}", data.u64[0]));
            break;
        case 0x4: // XYZF2
            gs_registers[4] = data.u64[0];
            Logger::info("XYZF2 register 0x" + format("{:016X}", data.u64[0]));
            break;
        case 0x5: // XYZ2
            gs_registers[5] = data.u64[0];
            Logger::info("XYZ2 register 0x" + format("{:016X}", data.u64[0]));
            break;
        case 0xA: // FOG
            gs_registers[10] = data.u64[0];
            Logger::info("FOG register 0x" + format("{:016X}", data.u64[0]));
            break;
        case 0xE: // A+D
        {
            uint32_t reg = data.u64[1];
            Logger::info("A+D register 0x" + format("{:016X}", data.u64[0]) + " redirecting write to internal register 0x" + format("{:X}", reg));
            write_internal_reg(reg, data.u64[0]);
            break;
        }
        default:
            Logger::error("Invalid packed GIF GS register address: 0x" + format("{:X}", reg));
            break;
    }
}

void GS::write_hwreg(uint64_t data) {
    hwreg = data;
    transfer_vram();
}

void GS::set_bitbltbuf(uint64_t value) {
    bitbltbuf = value;

    /*
        50h BITBLTBUF

        0-13    Source base pointer in words/64
        16-21   Source buffer width in pixels/64
        24-29   Source format
                00h=PSMCT32
                01h=PSMCT24
                02h=PSMCT16
                0Ah=PSMCT16S
                13h=PSMCT8
                14h=PSMCT4
                1Bh=PSMCT8H
                24h=PSMCT4HL
                2Ch=PSMCT4HH
                30h=PSMZ32
                31h=PSMZ24
                32h=PSMZ16
                3Ah=PSMZ16S
        32-45   Destination base pointer in words/64
        48-53   Destination buffer width in pixels/64
        56-61   Destination format (same as source format)
    */
    source_base_pointer         = (bitbltbuf >> 0) & 0x3fff;
    source_buffer_width         = (bitbltbuf >> 16) & 0x3f;
    source_format               = (bitbltbuf >> 24) & 0x3f;

    destination_base_pointer    = (bitbltbuf >> 32) & 0x3fff;
    destination_buffer_width    = (bitbltbuf >> 48) & 0x3f;
    destination_format          = (bitbltbuf >> 56) & 0x3f;

    source_base_pointer <<= 6;
    source_buffer_width <<= 6;

    destination_base_pointer <<= 6;
    destination_buffer_width <<= 6;

    Logger::info("BITBLTBUF set to 0x" + format("{:016X}", value));
}

void GS::set_trxpos(uint64_t value) {
    trxpos = value;

    /*
        51h TRXPOS

        0-10    X for source rectangle
        16-26   Y for source rectangle
        32-42   X for destination rectangle
        48-58   Y for destination rectangle
        59-60   Transmission order for VRAM->VRAM transfers
                0=Upper-left->lower-right
                1=Lower-left->upper-right
                2=Upper-right->lower-left
                3=Lower-right->upper-left
    */
    source_rectangle_x = (trxpos >> 0) & 0x7ff;
    source_rectangle_y = (trxpos >> 16) & 0x7ff;
    destination_rectangle_x = (trxpos >> 32) & 0x7ff;
    destination_rectangle_y = (trxpos >> 48) & 0x7ff;
    transmission_order = (trxpos >> 59) & 3;

    Logger::info("TRXPOS set to 0x" + format("{:016X}", value));
}

void GS::set_trxreg(uint64_t value) {
    trxreg = value;

    /*
        52h TRXREG

        0-11    Width in pixels of transmission area
        32-43   Height in pixels of transmission area
    */
    transmission_area_pixel_width = (trxreg >> 0) & 0xfff;
    transmission_area_pixel_height = (trxreg >> 32) & 0xfff;

    Logger::info("TRXREG set to 0x" + format("{:016X}", value));
}

void GS::set_trxdir(uint64_t value) {
    trxdir = value;

    /*
        53h TRXDIR

        0-1     Transmission direction
                0=GIF->VRAM
                1=VRAM->GIF
                2=VRAM->VRAM
                3=Deactivated
    */

    transmission_direction = trxdir & 3;
    source_x = 0;
    source_y = 0;
    destination_x = 0;
    destination_y = 0;

    if (transmission_direction == 2) {
        blit_vram();
    }
    else
    {
        Texture texture;
        texture.address = destination_base_pointer + destination_rectangle_x + (destination_rectangle_y * destination_buffer_width);
        texture.width = transmission_area_pixel_width;
        texture.height = transmission_area_pixel_height;
        texture.format = destination_format;
        texture.name = "Texture_" + std::to_string(textures.size());
        upload_texture(texture);
    }

    Logger::info("TRXDIR set to 0x" + format("{:016X}", value));
}

void GS::blit_vram() {
    for (uint32_t y = 0; y < transmission_area_pixel_height; y++) {
        uint32_t src = source_base_pointer + source_rectangle_x + (source_rectangle_y * source_buffer_width) + (y * source_buffer_width);
        uint32_t dst = destination_base_pointer + destination_rectangle_x + (destination_rectangle_y * destination_buffer_width) + (y * destination_buffer_width);

        if (src + transmission_area_pixel_width <= VRAM_SIZE && dst + transmission_area_pixel_width <= VRAM_SIZE) {
            memcpy(vram + dst, vram + src, transmission_area_pixel_width * sizeof(uint32_t));
        } else {
            Logger::error("VRAM blit out of bounds");
            Neo2::exit(1, Neo2::Subsystem::GS);
        }
    }
}

void GS::transfer_vram() {
    uint32_t address = destination_base_pointer + destination_rectangle_x + (destination_rectangle_y * destination_buffer_width);

    address += destination_x + (destination_y * destination_buffer_width);

    vram[address + 0] = hwreg & 0xffffffff;
    vram[address + 1] = hwreg >> 32;

    destination_x += 2;

    if (destination_x >= transmission_area_pixel_width) {
        destination_x = 0;

        destination_y++;
    }
}

void GS::upload_texture(const Texture& texture) {
    textures.push_back(texture);
    Logger::info("Texture uploaded: " + texture.name + " at address 0x" + format("{:08X}", texture.address));
}

const std::vector<GS::Texture>& GS::get_textures() const {
    return textures;
}