#include "neo2.hh"
#include <cmath>
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
#include <algorithm>

GS::GS() {
    for (int i = 0; i < 19; ++i) {
        gs_privileged_registers[i] = 0;
    }
    vram = new uint32_t[VRAM_SIZE]();
    Logger::set_subsystem("GS");

    // Initialize framebuffers
    framebuffer1.data = vram;
    framebuffer1.width = 640; // Default width, will be updated based on GS registers
    framebuffer1.height = 480; // Default height, will be updated based on GS registers
    framebuffer1.format = 0; // Default format, will be updated based on GS registers
    framebuffer1.fbw = 0;

    framebuffer2.data = vram;
    framebuffer2.width = 640; // Default width, will be updated based on GS registers
    framebuffer2.height = 480; // Default height, will be updated based on GS registers
    framebuffer2.format = 0; // Default format, will be updated based on GS registers
    framebuffer2.fbw = 0;

    current_prim = 0;
    vertex_count = 0;
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

    //Logger::info("GS register read from " + std::string(REGISTER_NAMES[index]));
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

    if (index == 15) {
        uint64_t old_value = gs_privileged_registers[index];
        uint64_t modified_bits = old_value ^ value;
        if (modified_bits & (1 << 3)) { // Check if bit 3 is being modified
            value &= ~(1 << 3); // Clear bit 3
        }
    }

    //Logger::info("GS register write to " + std::string(REGISTER_NAMES[index]) + " with value 0x" + format("{:08X}", value));
    gs_privileged_registers[index] = value;
}

void GS::simul_vblank() {
    //printf("Stepping VBlank\n");
    // Simulate GS operation complete by toggling bit 3
    gs_privileged_registers[15] |= 0x8; // Set bit 3
}

void GS::untog_vblank() {
    //printf("Stepping VBlank\n");
    // Simulate GS operation complete by toggling bit 3
    gs_privileged_registers[15] &= ~0x8; // Set bit 3
}

void GS::write_internal_reg(uint8_t reg, uint64_t data) {
    switch (reg) {
        case 0x00: // PRIM
        {
            //Logger::info("GIF: PRIM register 0x" + format("{:016X}", data.u64[0]));
            gs_registers[0x00] = data;
            handle_prim_selection(data);
            //Logger::info("PRIM register 0x" + format("{:016X}", data.u64[0]));
            break;
        }
        case 0x01: // RGBAQ
        {
            //Logger::info("GS: Writing to RGBAQ");
            //printf("Internal GS: RGBAQ -> %08X\n", data);
            uint64_t r = data & 0xFF;
            uint64_t g = (data >> 8) & 0xFF;
            uint64_t b = (data >> 16) & 0xFF;
            uint64_t a = (data >> 24) & 0xFF;
            uint64_t q = (data >> 32);
            uint64_t v = b | (g << 8) | (r << 16) | (a << 24) | (q << 32);
            gs_registers[0x01] = v;
        }
            break;
        case 0x02: // ST
            //Logger::info("GS: Writing to ST");
            gs_registers[0x02] = data;
            break;
        case 0x03: // UV
            gs_registers[0x03] = data;
            break;
        case 0x04: // XYZF2
        {
            //Logger::info("GS: Writing to XYZF2");
            //printf("Internal GS: XYZF2 -> %08X\n", data);
            uint64_t x = data & 0xffff;
            uint64_t y = (data >> 16) & 0xffff;
            uint64_t z = data >> 32;
            uint64_t v = x | (y << 16) | (z << 32);
            gs_registers[0x04] = v;
            add_vertex(v, true);
            break;
        }
        case 0x05: // XYZ2
        {
            //printf("Internal GS: XYZ2 -> %08X\n", data);
            //Logger::info("GS: Writing to XYZ2");
            uint64_t x = data & 0xffff;
            uint64_t y = (data >> 16) & 0xffff;
            uint64_t z = data >> 32;
            uint64_t v = x | (y << 16) | (z << 32);
            gs_registers[0x05] = data;
            add_vertex(v, true);
            break;
        }
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
        {
            uint64_t x = data & 0xffff;
            uint64_t y = (data >> 16) & 0xffff;
            uint64_t z = data >> 32;
            uint64_t v = x | (y << 16) | (z << 32);
            gs_registers[0x0C] = v;
            add_vertex(v, false);
            break;
        }
        case 0x0D: // XYZ3
        {
            uint64_t x = data & 0xffff;
            uint64_t y = (data >> 16) & 0xffff;
            uint64_t z = data >> 32;
            uint64_t v = x | (y << 16) | (z << 32);
            gs_registers[0x0D] = v;
            add_vertex(v, false);
            break;
        }
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
            //printf(BOLDYELLOW "XYOFFSET_1 write: 0x%016lX" RESET "\n", data);
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
            update_framebuffer(1, (data >> 16) & 0x3F, (data >> 32) & 0xFFFFFFFF, data & 0x3F);
            break;
        case 0x4D: // FRAME_2
            gs_registers[0x4D] = data;
            update_framebuffer(2, (data >> 16) & 0x3F, (data >> 32) & 0xFFFFFFFF, data & 0x3F);
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

int test = 0;

void GS::write_packed_gif_data(uint8_t reg, uint128_t data, uint32_t q) {
    // Handle the incoming GIF data and write to appropriate GS registers
    switch (reg) {
        case 0x0: // PRIM
        {
            //Logger::info("GIF: PRIM register 0x" + format("{:016X}", data.u64[0]));
            gs_registers[0x00] = data.u64[0] & 0x3ff;
            handle_prim_selection(data.u64[0] & 0x3ff);
            //Logger::info("PRIM register 0x" + format("{:016X}", data.u64[0]));
            break;
        }
        case 0x1: // RGBAQ
        {
            //Logger::info("GIF: RGBAQ register 0x" + format("{:016X}", data.u64[0]));
            //printf("PACKED GS: RGBAQ -> %016lX%016lX\n", data.u64[1], data.u64[0]);
            uint64_t r = data.u64[0] & 0xff;
            uint64_t g = (data.u64[0] >> 32) & 0xff;
            uint64_t b = data.u64[1] & 0xff;
            uint64_t a = (data.u64[1] >> 32) & 0xff;
            uint64_t v = (a << 24) | (r << 16) | (g << 8) | b;
            //printf("R: %d (%X), G: %d (%X), B: %d (%X), A: %d (%X) (%016lX)\n", r, r, g, g, b, b, a, a, v);
            gs_registers[1] = v;
            break;
        }
        case 0x2: // ST
        {
            q = data.u64[1] & 0xffffffff;
            gs_registers[2] = data.u64[0];
            //Logger::info("GIF: ST register 0x" + format("{:016X}", data.u64[0]));
            break;
        }
        case 0x3: // UV
            gs_registers[3] = (data.u64[0] & 0x3fff) | (data.u64[0] >> 16);
            //Logger::info("GIF: UV register 0x" + format("{:016X}", data.u64[0]));
            break;
        case 0x4: // XYZ2F/XYZ3F
        {
            //Logger::info("GIF: XYZF2 register 0x" + format("{:016X}", data.u64[0]));
            //printf(BOLDMAGENTA "PACKED GS: XYZF2 -> %016lX%016lX" RESET "\n", data.u64[1], data.u64[0]);
            uint64_t x = data.u64[0] & 0xffff;
            uint64_t y = (data.u64[0] >> 32) & 0xffff;
            uint64_t z = data.u64[1] >> 32;
            uint64_t v = x | (y << 16) | (z << 32);
            uint64_t disable_drawing = data.u64[1] & 0x800000000000ul;
            //printf(BOLDMAGENTA "X: %d (%X), Y: %d (%X), Z: %d (%X), V: (%016lX)" RESET "\n",
            //x, x, y, y, z, z, v);
            //printf(BOLDRED "data: %016lX" RESET "\n", v);
            add_vertex(v, disable_drawing ? false : true);
            //exit(1);
            gs_registers[disable_drawing ? 0x0C : 0x04] = v;
            break;
        }
        case 0x5: // XYZ2/XYZ3
        {
            //Logger::info("GIF: XYZ2 register 0x" + format("{:016X}", data.u64[0]));
            //printf("PACKED GS: XYZ2 -> %016lX%016lX\n", data.u64[1], data.u64[0]);
            uint64_t x = data.u64[0] & 0xffff;
            uint64_t y = (data.u64[0] >> 32) & 0xffff;
            uint64_t z = data.u64[1] >> 32;
            uint64_t v = x | (y << 16) | (z << 32);
            //printf("X: %d (%X), Y: %d (%X), Z: %d (%X)\n",
            //x, x, y, y, z, z);
            uint64_t disable_drawing = data.u64[1] & 0x800000000000ul;
            add_vertex(v, disable_drawing ? false : true);
            gs_registers[disable_drawing ? 0x0D : 0x05] = v;
            gs_registers[0x05] = v;
            break;
        }

        case 0x08:
            break;

        case 0x09:
            break;

        case 0xA: // FOG
            gs_registers[10] = data.u64[1] << 20;
            //Logger::info("FOG register 0x" + format("{:016X}", data.u64[0]));
            break;
        
        case 0xC:
            gs_registers[0x0C] = data.u64[0];
            break;

        case 0xE: // A+D
        {
            uint32_t reg = data.u64[1];
            //Logger::info("GIF: A+D register 0x" + format("{:016X}", data.u64[0]) + " redirecting write to internal register 0x" + format("{:X}", reg));
            write_internal_reg(reg, data.u64[0]);
            break;
        }

        case 0x0D:
            break;

        case 0x0F:
            break;
        default:
            Logger::error("Invalid packed GIF GS register address: 0x" + format("{:X}", reg));
            Neo2::exit(1, Neo2::Subsystem::GS);
            break;
    }
}

void GS::add_vertex(uint64_t data, bool vertex_kick) {
    //printf(BOLDBLUE "add_vertex -> data: %016lX" RESET" \n", data);

    PrimitiveType prim_type_ = static_cast<PrimitiveType>(current_prim & 0x7);

    if (current_primitive != prim_type_)
    {
        current_primitive = prim_type_;

        vertex_buffer.clear();
        vertex_count = 0;
        batch_draw();
    }

    int16_t x_fixed = static_cast<int16_t>((data >> 0) & 0xFFFF);
    int16_t y_fixed = static_cast<int16_t>((data >> 16) & 0xFFFF);
    //printf(BOLDBLUE "add_vertex -> x_fixed: %04X, y_fixed: %04X" RESET" \n", x_fixed, y_fixed);

    int16_t xoffset_1 = gs_registers[0x18] & 0xFFFF;
    int16_t yoffset_1 = (gs_registers[0x18] >> 32) & 0xFFFF;

    //printf(BOLDGREEN "add_vertex -> xoffset_1: %04X, yoffset_1: %04X" RESET" \n", xoffset_1, yoffset_1);

    x_fixed -= xoffset_1;
    y_fixed -= yoffset_1;

    //printf(BOLDCYAN "add_vertex -> x_fixed: %04X, y_fixed: %04X" RESET" \n", x_fixed, y_fixed);

    int32_t z = static_cast<int32_t>((data >> 48) & 0xFFFFFF);

    float x = static_cast<float>(x_fixed) / 16.0f; // Convert from 4-bit fractional fixed-point
    float y = static_cast<float>(y_fixed) / 16.0f; // Convert from 4-bit fractional fixed-point

    uint64_t rgbaq = gs_registers[0x01];
    uint8_t b = (rgbaq >> 0) & 0xFF;
    uint8_t g = (rgbaq >> 8) & 0xFF;
    uint8_t r = (rgbaq >> 16) & 0xFF;
    uint8_t a = (rgbaq >> 24) & 0xFF;
    uint32_t color = (r << 24) | (g << 16) | (b << 8) | a;
    //printf(BOLDBLUE "add_vertex: R: %d, G: %d, B: %d, x: %.0f, y: %.0f" RESET" \n", r, g, b, x, y);

    //printf(BOLDBLUE "add_vertex -> data: %016lX" RESET" \n", data);
    float u = static_cast<float>(gs_registers[0x02]);
    float v = static_cast<float>(gs_registers[0x03]);

    //if ((current_prim & 0x7) == 6) printf("Got vertex data: x: %.0f, y: %.0f\n", x, y);

    vertex_buffer.push_back({x, y, static_cast<float>(z), color, u, v});
    vertex_count++;

    uint32_t prim_type = current_prim & 0x7;
    bool should_draw = false;


    if (vertex_kick)
    {
        switch (prim_type) {
            case 0: // Point
                should_draw = (vertex_count >= 1);
                break;
            case 1: // Line
                should_draw = (vertex_count >= 2);
                break;
            case 3: // Triangle
                should_draw = (vertex_count >= 3);
                //if (should_draw) printf("Should draw triangle now, vertex's: %d\n", vertex_count);
                break;
            case 6: // Sprite
                should_draw = (vertex_count >= 2);
                //if (should_draw) printf("Should draw sprite now, vertex's: %d\n", vertex_count);
                break;
            default:
                Logger::error("Unsupported primitive type: " + std::to_string(prim_type));
                break;
        }

        if (should_draw) {
            PrimitiveType prim_type = static_cast<PrimitiveType>(current_prim & 0x7);

            Primitive prim;
            prim.type = static_cast<PrimitiveType>(prim_type);
            prim.vertices = vertex_buffer;  // Copy by value.
            {
                std::lock_guard<std::mutex> lock(prim_queue_mutex);
                prim_queue.push_back(prim);   
            }
            vertex_buffer.clear();
            vertex_count = 0;
        }
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

    //Logger::info("BITBLTBUF set to 0x" + format("{:016X}", value));
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

    //Logger::info("TRXPOS set to 0x" + format("{:016X}", value));
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

    //Logger::info("TRXREG set to 0x" + format("{:016X}", value));
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

    //Logger::info("TRXDIR set to 0x" + format("{:016X}", value));
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

void GS::handle_prim_selection(uint64_t prim) {
    current_prim = prim;
    vertex_count = 0;
}

void GS::draw_primitive() {
}

void GS::draw_point(const std::vector<Vertex>& vertices) {
    if (vertices.size() < 1)
    {
        Logger::error("Not enough vertices to draw point");
        return;
    }

    const Vertex& v = vertices[0];
    uint32_t x = static_cast<uint32_t>(v.x);
    uint32_t y = static_cast<uint32_t>(v.y);

    if (x < framebuffer1.width && y < framebuffer1.height) {
        vram[y * framebuffer1.width + x] = v.color;
    }
}

void GS::draw_line(const std::vector<Vertex>& vertices) {
    if (vertices.size() < 2)
    {
        Logger::error("Not enough vertices to draw line");
        return;
    }

    const Vertex& v1 = vertices[0];
    const Vertex& v2 = vertices[1];

    int x1 = static_cast<int>(v1.x);
    int y1 = static_cast<int>(v1.y);
    int x2 = static_cast<int>(v2.x);
    int y2 = static_cast<int>(v2.y);

    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x1 >= 0 && x1 < static_cast<int>(framebuffer1.width) && y1 >= 0 && y1 < static_cast<int>(framebuffer1.height)) {
            vram[y1 * framebuffer1.width + x1] = v1.color;
        }

        if (x1 == x2 && y1 == y2) break;
        int e2 = err * 2;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

int count = 0;

void GS::draw_triangle(const std::vector<Vertex>& vertices) {
    if (vertices.size() < 3) {
        Logger::error("Not enough vertices to draw triangle");
        return;
    }

    // Use the first three vertices for the triangle.
    const Vertex& v0 = vertices[0];
    const Vertex& v1 = vertices[1];
    const Vertex& v2 = vertices[2];

    // Compute the axis–aligned bounding box of the triangle.
    int minX = static_cast<int>(std::floor(std::min({ v0.x, v1.x, v2.x })));
    int maxX = static_cast<int>(std::ceil(std::max({ v0.x, v1.x, v2.x })));
    int minY = static_cast<int>(std::floor(std::min({ v0.y, v1.y, v2.y })));
    int maxY = static_cast<int>(std::ceil(std::max({ v0.y, v1.y, v2.y })));

    // Retrieve scissor values from the GS registers (assumed to be in SCISSOR_1).
    uint64_t scissor = gs_registers[0x40];
    int scax0 = scissor & 0x7ff;
    int scax1 = (scissor >> 16) & 0x7ff;
    int scay0 = (scissor >> 32) & 0x7ff;
    int scay1 = (scissor >> 48) & 0x7ff;

    // Clamp the bounding box to the scissor region.
    minX = std::max(minX, scax0);
    maxX = std::min(maxX, scax1);
    minY = std::max(minY, scay0);
    maxY = std::min(maxY, scay1);

    // Define a lambda for the edge function.
    auto edge = [](const Vertex &a, const Vertex &b, float x, float y) -> float {
        return (x - a.x) * (b.y - a.y) - (y - a.y) * (b.x - a.x);
    };

    // Compute the signed area of the triangle.
    float area = edge(v0, v1, v2.x, v2.y);
    if (area == 0.0f) {
        // Degenerate triangle; nothing to draw.
        return;
    }

    // Retrieve z–buffer parameters from the GS registers (assumed to be in ZBUF_1).
    uint64_t zbuf = gs_registers[0x4E];
    uint32_t zbuf_base = (zbuf & 0x1FF) << 11;
    uint32_t zbuf_format = (zbuf >> 24) & 0xF;
    bool zbuf_mask = (zbuf >> 32) & 0x1;

    // Loop over each pixel in the bounding box.
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            // Compute barycentric weights at the center of the pixel.
            float w0 = edge(v1, v2, x + 0.5f, y + 0.5f);
            float w1 = edge(v2, v0, x + 0.5f, y + 0.5f);
            float w2 = edge(v0, v1, x + 0.5f, y + 0.5f);

            // If the triangle area is negative, reverse the weights.
            if (area < 0) {
                w0 = -w0;
                w1 = -w1;
                w2 = -w2;
            }

            // If the point is inside (or exactly on the edge of) the triangle.
            if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                // Normalize the barycentrics.
                w0 /= area;
                w1 /= area;
                w2 /= area;

                // Interpolate the z–value (depth) for this pixel.
                float zf = w0 * v0.z + w1 * v1.z + w2 * v2.z;
                uint32_t z_value = static_cast<uint32_t>(zf);

                // Compute the z–buffer index.
                uint32_t z_index = zbuf_base + y * framebuffer1.width + x;

                // Perform z–buffer testing and write the pixel if the test passes.
                if (!zbuf_mask) {
                    if (zbuf_format == 0x0 || zbuf_format == 0x1) { // PSMZ32 or PSMZ24
                        if (z_value >= vram[z_index]) {
                            vram[z_index] = z_value;
                            // Write a flat color (using v0.color); you could also interpolate color.
                            vram[y * framebuffer1.fbw + x] = ((v1.color & 0x000000FF) << 24) | // B to XBGR's B
                                                             ((v1.color & 0x0000FF00) << 8)  | // G to XBGR's G
                                                             ((v1.color & 0x00FF0000) >> 8)  | // R to XBGR's R
                                                             ((v1.color & 0xFF000000) >> 24);
                        }
                    } else if (zbuf_format == 0x2 || zbuf_format == 0xA) { // PSMZ16 or PSMZ16S
                        uint16_t* zbuf16 = reinterpret_cast<uint16_t*>(vram);
                        if (z_value >= zbuf16[z_index]) {
                            zbuf16[z_index] = static_cast<uint16_t>(z_value);
                            vram[y * framebuffer1.fbw + x] = ((v1.color & 0x000000FF) << 24) | // B to XBGR's B
                                                             ((v1.color & 0x0000FF00) << 8)  | // G to XBGR's G
                                                             ((v1.color & 0x00FF0000) >> 8)  | // R to XBGR's R
                                                             ((v1.color & 0xFF000000) >> 24);
                        }
                    }
                } else {
                    // If the z–buffer is disabled, simply write the pixel.
                    vram[y * framebuffer1.fbw + x] = ((v1.color & 0x000000FF) << 24) | // B to XBGR's B
                                                     ((v1.color & 0x0000FF00) << 8)  | // G to XBGR's G
                                                     ((v1.color & 0x00FF0000) >> 8)  | // R to XBGR's R
                                                     ((v1.color & 0xFF000000) >> 24);
                }
            }
        }
    }
}

void GS::draw_sprite(const std::vector<Vertex>& vertices) {
    if (vertices.size() < 2) {
        Logger::error("Not enough vertices to draw sprite");
        return;
    }

    // Use the first two vertices for the sprite.
    // They typically represent two opposite corners of the sprite rectangle.
    const Vertex& v0 = vertices[0];
    const Vertex& v1 = vertices[1];

    // Compute the axis–aligned bounding box of the sprite.
    int minX = static_cast<int>(std::floor(std::min(v0.x, v1.x)));
    int maxX = static_cast<int>(std::ceil(std::max(v0.x, v1.x)));
    int minY = static_cast<int>(std::floor(std::min(v0.y, v1.y)));
    int maxY = static_cast<int>(std::ceil(std::max(v0.y, v1.y)));

    // Retrieve scissor values from the GS registers (assumed to be in SCISSOR_1).
    uint64_t scissor = gs_registers[0x40];
    int scax0 = scissor & 0x7ff;
    int scax1 = (scissor >> 16) & 0x7ff;
    int scay0 = (scissor >> 32) & 0x7ff;
    int scay1 = (scissor >> 48) & 0x7ff;

    // Clamp the bounding box to the scissor region.
    minX = std::max(minX, scax0);
    maxX = std::min(maxX, scax1);
    minY = std::max(minY, scay0);
    maxY = std::min(maxY, scay1);

    // Retrieve z–buffer parameters from the GS registers (assumed to be in ZBUF_1).
    uint64_t zbuf = gs_registers[0x4E];
    uint32_t zbuf_base = (zbuf & 0x1FF) << 11;
    uint32_t zbuf_format = (zbuf >> 24) & 0xF;
    bool zbuf_mask = (zbuf >> 32) & 0x1;

    // For sprite rendering we use a constant depth.
    // Here we take the depth from v1 (you could choose v0 or average the two).
    uint32_t sprite_z = static_cast<uint32_t>(v1.z);

    // Convert the color from v1 to XBGR order.
    uint32_t converted_color = ((v1.color & 0x000000FF) << 24) | // B -> XBGR's B
                               ((v1.color & 0x0000FF00) << 8)  | // G -> XBGR's G
                               ((v1.color & 0x00FF0000) >> 8)  | // R -> XBGR's R
                               ((v1.color & 0xFF000000) >> 24);   // A remains in place

    // Loop over each pixel in the bounding rectangle.
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {

            // Compute the z–buffer index.
            uint32_t z_index = zbuf_base + y * framebuffer1.width + x;

            // Perform z–buffer testing and write the pixel if the test passes.
            if (!zbuf_mask) {
                if (zbuf_format == 0x0 || zbuf_format == 0x1) { // PSMZ32 or PSMZ24
                    if (sprite_z >= vram[z_index]) {
                        vram[z_index] = sprite_z;
                        vram[y * framebuffer1.fbw + x] = converted_color;
                    }
                } else if (zbuf_format == 0x2 || zbuf_format == 0xA) { // PSMZ16 or PSMZ16S
                    uint16_t* zbuf16 = reinterpret_cast<uint16_t*>(vram);
                    if (sprite_z >= zbuf16[z_index]) {
                        zbuf16[z_index] = static_cast<uint16_t>(sprite_z);
                        vram[y * framebuffer1.fbw + x] = converted_color;
                    }
                }
            } else {
                // If z–buffering is disabled, write the pixel directly.
                vram[y * framebuffer1.fbw + x] = converted_color;
            }
        }
    }
}


void GS::update_framebuffer(uint32_t frame, uint32_t width, uint32_t height, uint32_t format) {
    int pmode_idx = 0; // Index for PMODE register
    int display1_idx = 8; // Index for DISPLAY1 register
    int display2_idx = 10; // Index for DISPLAY2 register
    int dispfb1_idx = 7; // Index for DISPFB1 register
    int dispfb2_idx = 9; // Index for DISPFB2 register
    int scissor1_idx = 0x40; // Index for SCISSOR_1 register
    int scissor2_idx = 0x41; // Index for SCISSOR_2 register
    int frame1_idx = 0x4C; // Index for SCISSOR_1 register
    int frame2_idx = 0x4D; // Index for SCISSOR_2 register

    int en1 = gs_privileged_registers[pmode_idx] & 1;
    int en2 = (gs_privileged_registers[pmode_idx] >> 1) & 1;

    uint64_t display = en1 ? gs_privileged_registers[display1_idx] : gs_privileged_registers[display2_idx];
    uint64_t dispfb = en1 ? gs_privileged_registers[dispfb1_idx] : gs_privileged_registers[dispfb2_idx];

    uint64_t scissor = en1 ? gs_registers[scissor1_idx] : gs_registers[scissor2_idx];
    int scax0 = scissor & 0x7ff;
    int scax1 = (scissor >> 16) & 0x7ff;
    int scay0 = (scissor >> 32) & 0x7ff;
    int scay1 = (scissor >> 48) & 0x7ff;

    int sw = (scax1 - scax0) + 1;
    int sh = (scay1 - scay0) + 1;
    int magh = ((display >> 23) & 7) + 1;
    int magv = ((display >> 27) & 3) + 1;
    int dw = ((display >> 32) & 0xfff) / magh;
    int dh = ((display >> 44) & 0x7ff) / magv;

    uint64_t frame1_fbw = ((gs_registers[frame1_idx]) >> 16) & 0x3F;

    if (frame == 1) {
        framebuffer1.width = dw;
        framebuffer1.height = dh;
        framebuffer1.format = format;
        framebuffer1.fbw = frame1_fbw * 64;
        framebuffer1.data = vram; // Assuming framebuffer1 uses the same VRAM
        //Logger::info("Framebuffer 1 updated: width=" + std::to_string(width) + ", height=" + std::to_string(height) + ", format=" + std::to_string(format));
    } else if (frame == 2) {
        framebuffer2.width = dw;
        framebuffer2.height = dh;
        framebuffer2.format = format;
        framebuffer2.data = vram; // Assuming framebuffer2 uses the same VRAM
        //Logger::info("Framebuffer 2 updated: width=" + std::to_string(width) + ", height=" + std::to_string(height) + ", format=" + std::to_string(format));
    }
}

int num = 0;
void GS::batch_draw() {
    bool executed = false;
    {
        std::lock_guard<std::mutex> lock(prim_queue_mutex);
        for (const Primitive& prim : prim_queue) {
            executed = true;
            switch (prim.type) {
                case PrimitiveType::Triangle:
                    draw_triangle(prim.vertices);
                    break;
                case PrimitiveType::Sprite:
                    draw_sprite(prim.vertices);
                    break;
                default:
                    Logger::error("Unsupported primitive type in batch_draw");
                    Neo2::exit(1, Neo2::Subsystem::GS);
                    break;
            }
        }
        prim_queue.clear();
    }
    if (executed) {
        vertex_buffer.clear();
        vertex_count = 0;
    }
}
