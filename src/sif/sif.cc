#include <sif/sif.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif
#include <neo2.hh>

SIF::SIF() {
    Logger::set_subsystem("SIF");
    mscom = 0;
    smcom = 0;
    msflg = 0;
    smflg = 0;
    ctrl = 0;
    bd6 = 0;
}

uint32_t SIF::read(uint32_t address) {
    std::string reg_name;

    switch (address & 0xFF) {
        case 0x00:
            reg_name = "SIF_MSCOM";
            Logger::info("SIF register read from " + reg_name);
            return mscom;

        case 0x10:
            reg_name = "SIF_SMCOM";
            Logger::info("SIF register read from " + reg_name);
            return smcom;

        case 0x20:
            reg_name = "SIF_MSFLG";
            Logger::info("SIF register read from " + reg_name);
            return msflg;

        case 0x30:
            reg_name = "SIF_SMFLG";
            Logger::info("SIF register read from " + reg_name);
            return smflg;

        case 0x40:
            reg_name = "SIF_CTRL";
            Logger::info("SIF register read from " + reg_name);
            return ctrl;

        case 0x60:
            reg_name = "SIF_BD6";
            Logger::info("SIF register read from " + reg_name);
            return bd6;

        default:
            Logger::error("Invalid SIF register read at address 0x" + format("{:08X}", address));
            break;
    }

    return Neo2::exit(1, Neo2::Subsystem::IOP);
}

void SIF::write(uint32_t address, uint32_t value) {
    std::string reg_name;
    bool is_ee = (address & 0x0F000000) == 0x0F000000;

    switch (address & 0xFF) {
        case 0x00:
            if (!is_ee) {
                Logger::error("SIF_MSCOM is only writable by EE");
                return;
            }
            reg_name = "SIF_MSCOM";
            Logger::info("SIF register write to " + reg_name);
            mscom = value;
            break;

        case 0x10:
            if (is_ee) {
                Logger::error("SIF_SMCOM is only writable by IOP");
                return;
            }
            reg_name = "SIF_SMCOM";
            Logger::info("SIF register write to " + reg_name);
            smcom = value;
            break;

        case 0x20:
            if (is_ee) {
                Logger::error("SIF_MSFLG is only writable by IOP");
                return;
            }
            reg_name = "SIF_MSFLG";
            Logger::info("SIF register write to " + reg_name);
            msflg = value;
            break;

        case 0x30:
            if (!is_ee) {
                Logger::error("SIF_SMFLG is only writable by EE");
                return;
            }
            reg_name = "SIF_SMFLG";
            Logger::info("SIF register write to " + reg_name);
            smflg = value;
            break;

        case 0x40:
            reg_name = "SIF_CTRL";
            Logger::info("SIF register write to " + reg_name);
            // Apply rules for SIF_CTRL register
            value |= 0xF0000000; // Bits 28-31 always 0xF
            if (!is_ee) {
                value &= ~0x100; // Bit 8 always 0 for IOP
            }
            value |= 0x1; // Bit 1 always 1
            ctrl = value;
            break;

        case 0x60:
            reg_name = "SIF_BD6";
            Logger::info("SIF register write to " + reg_name);
            bd6 = value;
            break;

        default:
            Logger::error("Invalid SIF register write at address 0x" + format("{:08X}", address));
            break;
    }
}