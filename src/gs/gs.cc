#include <gs/gs.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

GS::GS() {
    for (int i = 0; i < 19; ++i) {
        gs_privileged_registers[i] = 0;
    }
    Logger::set_subsystem("GS");
}

uint64_t GS::read(uint32_t address) {
    std::string reg_name;
    switch (address) {
        case PMODE:
            reg_name = "PMODE";
            break;
        case SMODE1:
            reg_name = "SMODE1";
            break;
        case SMODE2:
            reg_name = "SMODE2";
            break;
        case SRFSH:
            reg_name = "SRFSH";
            break;
        case SYNCH1:
            reg_name = "SYNCH1";
            break;
        case SYNCH2:
            reg_name = "SYNCH2";
            break;
        case SYNCV:
            reg_name = "SYNCV";
            break;
        case DISPFB1:
            reg_name = "DISPFB1";
            break;
        case DISPLAY1:
            reg_name = "DISPLAY1";
            break;
        case DISPFB2:
            reg_name = "DISPFB2";
            break;
        case DISPLAY2:
            reg_name = "DISPLAY2";
            break;
        case EXTBUF:
            reg_name = "EXTBUF";
            break;
        case EXTDATA:
            reg_name = "EXTDATA";
            break;
        case EXTWRITE:
            reg_name = "EXTWRITE";
            break;
        case BGCOLOR:
            reg_name = "BGCOLOR";
            break;
        case GS_CSR:
            reg_name = "GS_CSR";
            break;
        case GS_IMR:
            reg_name = "GS_IMR";
            break;
        case BUSDIR:
            reg_name = "BUSDIR";
            break;
        case SIGLBLID:
            reg_name = "SIGLBLID";
            break;
        default:
            Logger::error("Invalid GS register read at address 0x" + format("{:08X}", address));
            return 0;
    }
    Logger::info("GS register read from " + reg_name);
    return gs_privileged_registers[(address - 0x12000000) / 0x10];
}

void GS::write(uint32_t address, uint64_t value) {
    std::string reg_name;
    switch (address) {
        case PMODE:
            reg_name = "PMODE";
            break;
        case SMODE1:
            reg_name = "SMODE1";
            break;
        case SMODE2:
            reg_name = "SMODE2";
            break;
        case SRFSH:
            reg_name = "SRFSH";
            break;
        case SYNCH1:
            reg_name = "SYNCH1";
            break;
        case SYNCH2:
            reg_name = "SYNCH2";
            break;
        case SYNCV:
            reg_name = "SYNCV";
            break;
        case DISPFB1:
            reg_name = "DISPFB1";
            break;
        case DISPLAY1:
            reg_name = "DISPLAY1";
            break;
        case DISPFB2:
            reg_name = "DISPFB2";
            break;
        case DISPLAY2:
            reg_name = "DISPLAY2";
            break;
        case EXTBUF:
            reg_name = "EXTBUF";
            break;
        case EXTDATA:
            reg_name = "EXTDATA";
            break;
        case EXTWRITE:
            reg_name = "EXTWRITE";
            break;
        case BGCOLOR:
            reg_name = "BGCOLOR";
            break;
        case GS_CSR:
            reg_name = "GS_CSR";
            break;
        case GS_IMR:
            reg_name = "GS_IMR";
            break;
        case BUSDIR:
            reg_name = "BUSDIR";
            break;
        case SIGLBLID:
            reg_name = "SIGLBLID";
            break;
        default:
            Logger::error("Invalid GS register write at address 0x" + format("{:08X}", address));
            return;
    }
    Logger::info("GS register write to " + reg_name + " with value 0x" + format("{:08X}", value));
    gs_privileged_registers[(address - 0x12000000) / 0x10] = value;
}

void GS::simul_vblank() {
    // Simulate GS operation complete by toggling bit 3
    gs_privileged_registers[(GS_CSR - 0x12000000) / 0x10] |= 0x8; // Set bit 3
}

void GS::write_internal_reg(uint8_t reg, uint64_t data) {
    // Handle the incoming GIF data and write to appropriate GS registers
    switch (reg) {
        case 0x0: // PRIM
            gs_registers[0] = data;
            Logger::info("PRIM register 0x" + format("{:016X}", data));
            break;
        case 0x1: // RGBAQ
            gs_registers[1] = data;
            Logger::info("RGBAQ register 0x" + format("{:016X}", data));
            break;
        case 0x2: // ST
            gs_registers[2] = data;
            Logger::info("ST register 0x" + format("{:016X}", data));
            break;
        case 0x3: // UV
            gs_registers[3] = data;
            Logger::info("UV register 0x" + format("{:016X}", data));
            break;
        case 0x4: // XYZF2
            gs_registers[4] = data;
            Logger::info("XYZF2 register 0x" + format("{:016X}", data));
            break;
        case 0x5: // XYZ2
            gs_registers[5] = data;
            Logger::info("XYZ2 register 0x" + format("{:016X}", data));
            break;
        case 0x6: // TEX0_1
            gs_registers[6] = data;
            Logger::info("TEX0_1 register 0x" + format("{:016X}", data));
            break;
        case 0x7: // TEX0_2
            gs_registers[7] = data;
            Logger::info("TEX0_2 register 0x" + format("{:016X}", data));
            break;
        case 0x8: // CLAMP_1
            gs_registers[8] = data;
            Logger::info("CLAMP_1 register 0x" + format("{:016X}", data));
            break;
        case 0x9: // CLAMP_2
            gs_registers[9] = data;
            Logger::info("CLAMP_2 register 0x" + format("{:016X}", data));
            break;
        case 0xA: // FOG
            gs_registers[10] = data;
            Logger::info("FOG register 0x" + format("{:016X}", data));
            break;
        case 0xC: // XYZF3
            gs_registers[12] = data;
            Logger::info("XYZF3 register 0x" + format("{:016X}", data));
            break;
        case 0xD: // XYZ3
            gs_registers[13] = data;
            Logger::info("XYZ3 register 0x" + format("{:016X}", data));
            break;
        case 0x14: // TEX1_1
            gs_registers[14] = data;
            Logger::info("TEX1_1 register 0x" + format("{:016X}", data));
            break;
        case 0x15: // TEX1_2
            gs_registers[15] = data;
            Logger::info("TEX1_2 register 0x" + format("{:016X}", data));
            break;
        case 0x16: // TEX2_1
            gs_registers[16] = data;
            Logger::info("TEX2_1 register 0x" + format("{:016X}", data));
            break;
        case 0x17: // TEX2_2
            gs_registers[17] = data;
            Logger::info("TEX2_2 register 0x" + format("{:016X}", data));
            break;
        case 0x18: // XYOFFSET_1
            gs_registers[18] = data;
            Logger::info("XYOFFSET_1 register 0x" + format("{:016X}", data));
            break;
        case 0x19: // XYOFFSET_2
            gs_registers[19] = data;
            Logger::info("XYOFFSET_2 register 0x" + format("{:016X}", data));
            break;
        case 0x1A: // PRMODECONT
            gs_registers[20] = data;
            Logger::info("PRMODECONT register 0x" + format("{:016X}", data));
            break;
        case 0x1B: // PRMODE
            gs_registers[21] = data;
            Logger::info("PRMODE register 0x" + format("{:016X}", data));
            break;
        case 0x1C: // TEXCLUT
            gs_registers[22] = data;
            Logger::info("TEXCLUT register 0x" + format("{:016X}", data));
            break;
        case 0x22: // SCANMSK
            gs_registers[23] = data;
            Logger::info("SCANMSK register 0x" + format("{:016X}", data));
            break;
        case 0x34: // MIPTBP1_1
            gs_registers[24] = data;
            Logger::info("MIPTBP1_1 register 0x" + format("{:016X}", data));
            break;
        case 0x35: // MIPTBP1_2
            gs_registers[25] = data;
            Logger::info("MIPTBP1_2 register 0x" + format("{:016X}", data));
            break;
        case 0x36: // MIPTBP2_1
            gs_registers[26] = data;
            Logger::info("MIPTBP2_1 register 0x" + format("{:016X}", data));
            break;
        case 0x37: // MIPTBP2_2
            gs_registers[27] = data;
            Logger::info("MIPTBP2_2 register 0x" + format("{:016X}", data));
            break;
        case 0x3B: // TEXA
            gs_registers[28] = data;
            Logger::info("TEXA register 0x" + format("{:016X}", data));
            break;
        case 0x3D: // FOGCOL
            gs_registers[29] = data;
            Logger::info("FOGCOL register 0x" + format("{:016X}", data));
            break;
        case 0x3F: // TEXFLUSH
            gs_registers[30] = data;
            Logger::info("TEXFLUSH register 0x" + format("{:016X}", data));
            break;
        case 0x40: // SCISSOR_1
            gs_registers[31] = data;
            Logger::info("SCISSOR_1 register 0x" + format("{:016X}", data));
            break;
        case 0x41: // SCISSOR_2
            gs_registers[32] = data;
            Logger::info("SCISSOR_2 register 0x" + format("{:016X}", data));
            break;
        case 0x42: // ALPHA_1
            gs_registers[33] = data;
            Logger::info("ALPHA_1 register 0x" + format("{:016X}", data));
            break;
        case 0x43: // ALPHA_2
            gs_registers[34] = data;
            Logger::info("ALPHA_2 register 0x" + format("{:016X}", data));
            break;
        case 0x44: // DIMX
            gs_registers[35] = data;
            Logger::info("DIMX register 0x" + format("{:016X}", data));
            break;
        case 0x45: // DTHE
            gs_registers[36] = data;
            Logger::info("DTHE register 0x" + format("{:016X}", data));
            break;
        case 0x46: // COLCLAMP
            gs_registers[37] = data;
            Logger::info("COLCLAMP register 0x" + format("{:016X}", data));
            break;
        case 0x47: // TEST_1
            gs_registers[38] = data;
            Logger::info("TEST_1 register 0x" + format("{:016X}", data));
            break;
        case 0x48: // TEST_2
            gs_registers[39] = data;
            Logger::info("TEST_2 register 0x" + format("{:016X}", data));
            break;
        case 0x49: // PABE
            gs_registers[40] = data;
            Logger::info("PABE register 0x" + format("{:016X}", data));
            break;
        case 0x4A: // FBA_1
            gs_registers[41] = data;
            Logger::info("FBA_1 register 0x" + format("{:016X}", data));
            break;
        case 0x4B: // FBA_2
            gs_registers[42] = data;
            Logger::info("FBA_2 register 0x" + format("{:016X}", data));
            break;
        case 0x4C: // FRAME_1
            gs_registers[43] = data;
            Logger::info("FRAME_1 register 0x" + format("{:016X}", data));
            break;
        case 0x4D: // FRAME_2
            gs_registers[44] = data;
            Logger::info("FRAME_2 register 0x" + format("{:016X}", data));
            break;
        case 0x4E: // ZBUF_1
            gs_registers[45] = data;
            Logger::info("ZBUF_1 register 0x" + format("{:016X}", data));
            break;
        case 0x4F: // ZBUF_2
            gs_registers[46] = data;
            Logger::info("ZBUF_2 register 0x" + format("{:016X}", data));
            break;
        case 0x50: // BITBLTBUF
            gs_registers[47] = data;
            Logger::info("BITBLTBUF register 0x" + format("{:016X}", data));
            break;
        case 0x51: // TRXPOS
            gs_registers[48] = data;
            Logger::info("TRXPOS register 0x" + format("{:016X}", data));
            break;
        case 0x52: // TRXREG
            gs_registers[49] = data;
            Logger::info("TRXREG register 0x" + format("{:016X}", data));
            break;
        case 0x53: // TRXDIR
            gs_registers[50] = data;
            Logger::info("TRXDIR register 0x" + format("{:016X}", data));
            break;
        case 0x54: // HWREG
            gs_registers[51] = data;
            Logger::info("HWREG register 0x" + format("{:016X}", data));
            break;
        case 0x60: // SIGNAL
            gs_registers[52] = data;
            Logger::info("SIGNAL register 0x" + format("{:016X}", data));
            break;
        case 0x61: // FINISH
            gs_registers[53] = data;
            Logger::info("FINISH register 0x" + format("{:016X}", data));
            break;
        case 0x62: // LABEL
            gs_registers[54] = data;
            Logger::info("LABEL register 0x" + format("{:016X}", data));
            break;
        default:
            Logger::error("Invalid GS register address: 0x" + format("{:X}", reg));
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