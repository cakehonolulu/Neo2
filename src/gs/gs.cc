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
    vram = new uint32_t[VRAM_SIZE / sizeof(uint32_t)]();
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
    gs_privileged_registers[0x12] |= 0x8; // Set bit 3
}

void GS::write_internal_reg(uint8_t reg, uint64_t data) {
    switch (reg) {
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
        default:
            if (reg < 55) {
                gs_registers[reg] = data;
                Logger::info(std::string(INTERNAL_REGISTER_NAMES[reg]) + " register 0x" + format("{:016X}", data));
            } else {
                Logger::error("Invalid internal GS register address: 0x" + format("{:X}", reg));
            }
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
        Logger::error("VRAM to VRAM unimpl.");
        Neo2::exit(1, Neo2::Subsystem::GS);
    }

    Logger::info("TRXDIR set to 0x" + format("{:016X}", value));
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