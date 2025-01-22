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
        gs_registers[i] = 0;
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
    return gs_registers[(address - 0x12000000) / 0x10];
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
    gs_registers[(address - 0x12000000) / 0x10] = value;
}
