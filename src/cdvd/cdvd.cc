#include <cdvd/cdvd.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif
#include <neo2.hh>

CDVD::CDVD() {
    Logger::set_subsystem("CDVD");
    current_n_command = 0;
    n_command_status = 0x4c;
    n_command_param = 0;
    cdvd_error = 0;
    i_stat = 0;
    drive_status = 0;
    sticky_drive_status = 0x1e;
    disk_type = 0;
    current_s_command = 0;
    s_command_status = 64;
    s_command_param = 0;
    s_command_result = 0;
}

uint32_t CDVD::read(uint32_t address) {
    std::string reg_name;

    switch (address & 0xFF) {
        case 0x04:
            reg_name = "Current N command";
            Logger::info("CDVD register read from " + reg_name);
            return current_n_command;

        case 0x05:
            reg_name = "N command status";
            Logger::info("CDVD register read from " + reg_name);
            return n_command_status;

        case 0x06:
            reg_name = "CDVD error";
            Logger::info("CDVD register read from " + reg_name);
            return cdvd_error;

        case 0x08:
            reg_name = "CDVD I_STAT";
            Logger::info("CDVD register read from " + reg_name);
            return i_stat;

        case 0x0A:
            reg_name = "CDVD drive status";
            Logger::info("CDVD register read from " + reg_name);
            return drive_status;

        case 0x0B:
            reg_name = "Sticky drive status";
            Logger::info("CDVD register read from " + reg_name);
            return sticky_drive_status;

        case 0x0F:
            reg_name = "CDVD disk type";
            Logger::info("CDVD register read from " + reg_name);
            return disk_type;

        case 0x16:
            reg_name = "Current S command";
            Logger::info("CDVD register read from " + reg_name);
            return current_s_command;

        case 0x17:
            reg_name = "S command status";
            Logger::info("CDVD register read from " + reg_name);
            return s_command_status;

        case 0x18:
            reg_name = "S command result";
            Logger::info("CDVD register read from " + reg_name);
            return s_command_result;

        default:
            Logger::error("Invalid CDVD register read at address 0x" + format("{:08X}", address));
            break;
    }

    return Neo2::exit(1, Neo2::Subsystem::IOP);
}

void CDVD::write(uint32_t address, uint32_t value) {
    std::string reg_name;

    switch (address & 0xFF) {
        case 0x04:
            reg_name = "Current N command";
            Logger::info("CDVD register write to " + reg_name);
            current_n_command = value;
            break;

        case 0x05:
            reg_name = "N command param";
            Logger::info("CDVD register write to " + reg_name);
            n_command_param = value;
            break;

        case 0x07:
            reg_name = "BREAK";
            Logger::info("CDVD register write to " + reg_name);
            // Handle BREAK command
            break;

        case 0x08:
            reg_name = "CDVD I_STAT";
            Logger::info("CDVD register write to " + reg_name);
            i_stat = value;
            break;

        case 0x16:
            reg_name = "Current S command";
            Logger::info("CDVD register write to " + reg_name);
            current_s_command = value;
            break;

        case 0x17:
            reg_name = "S command params";
            Logger::info("CDVD register write to " + reg_name);
            s_command_param = value;
            break;

        default:
            Logger::error("Invalid CDVD register write at address 0x" + format("{:08X}", address));
            break;
    }
}