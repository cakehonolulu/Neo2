#include <iop/iop_sio2.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif
#include <neo2.hh>

IOP_SIO2::IOP_SIO2() {
    Logger::set_subsystem("IOP_SIO2");
    std::memset(send3, 0, sizeof(send3));
    std::memset(send1, 0, sizeof(send1));
    std::memset(send2, 0, sizeof(send2));
    fifoin = 0;
    fifoout = 0;
    ctrl = 0x3BC; // Initial value on PS2 reset
    recv1 = 0;
    recv2 = 0;
    recv3 = 0;
    istat = 0;
}

uint32_t IOP_SIO2::read(uint32_t address) {
    std::string reg_name;

    switch (address & 0x3F) {
        case 0x00 ... 0x3F:
            reg_name = format("SIO2_SEND3[{}]", (address & 0x3F) / 4);
            Logger::info("IOP SIO2 register read from " + reg_name);
            return send3[(address & 0x3F) / 4];

        case 0x40 ... 0x5F:
            reg_name = format("SIO2_SEND1/2[{}]", (address & 0x1F) / 4);
            Logger::info("IOP SIO2 register read from " + reg_name);
            return (address & 0x04) ? send2[(address & 0x1F) / 4] : send1[(address & 0x1F) / 4];

        case 0x60:
            reg_name = "SIO2_FIFOIN";
            Logger::info("IOP SIO2 register read from " + reg_name);
            return fifoin;

        case 0x64:
            reg_name = "SIO2_FIFOOUT";
            Logger::info("IOP SIO2 register read from " + reg_name);
            return fifoout;

        case 0x68:
            reg_name = "SIO2_CTRL";
            Logger::info("IOP SIO2 register read from " + reg_name);
            return ctrl;

        case 0x6C:
            reg_name = "SIO2_RECV1";
            Logger::info("IOP SIO2 register read from " + reg_name);
            return recv1;

        case 0x70:
            reg_name = "SIO2_RECV2";
            Logger::info("IOP SIO2 register read from " + reg_name);
            return recv2;

        case 0x74:
            reg_name = "SIO2_RECV3";
            Logger::info("IOP SIO2 register read from " + reg_name);
            return recv3;

        case 0x80:
            reg_name = "SIO2_ISTAT";
            Logger::info("IOP SIO2 register read from " + reg_name);
            return istat;

        default:
            Logger::error("Invalid IOP SIO2 register read at address 0x" + format("{:08X}", address));
            break;
    }

    return Neo2::exit(1, Neo2::Subsystem::IOP);
}

void IOP_SIO2::write(uint32_t address, uint32_t value) {
    std::string reg_name;

    switch (address & 0x3F) {
        case 0x00 ... 0x3F:
            reg_name = format("SIO2_SEND3[{}]", (address & 0x3F) / 4);
            Logger::info("IOP SIO2 register write to " + reg_name);
            send3[(address & 0x3F) / 4] = value;
            break;

        case 0x40 ... 0x5F:
            reg_name = format("SIO2_SEND1/2[{}]", (address & 0x1F) / 4);
            Logger::info("IOP SIO2 register write to " + reg_name);
            if (address & 0x04) {
                send2[(address & 0x1F) / 4] = value;
            } else {
                send1[(address & 0x1F) / 4] = value;
            }
            break;

        case 0x60:
            reg_name = "SIO2_FIFOIN";
            Logger::info("IOP SIO2 register write to " + reg_name);
            fifoin = value;
            break;

        case 0x64:
            reg_name = "SIO2_FIFOOUT";
            Logger::info("IOP SIO2 register write to " + reg_name);
            fifoout = value;
            break;

        case 0x68:
            reg_name = "SIO2_CTRL";
            Logger::info("IOP SIO2 register write to " + reg_name);
            ctrl = value;
            break;

        case 0x6C:
            reg_name = "SIO2_RECV1";
            Logger::info("IOP SIO2 register write to " + reg_name);
            recv1 = value;
            break;

        case 0x70:
            reg_name = "SIO2_RECV2";
            Logger::info("IOP SIO2 register write to " + reg_name);
            recv2 = value;
            break;

        case 0x74:
            reg_name = "SIO2_RECV3";
            Logger::info("IOP SIO2 register write to " + reg_name);
            recv3 = value;
            break;

        case 0x80:
            reg_name = "SIO2_ISTAT";
            Logger::info("IOP SIO2 register write to " + reg_name);
            istat = value;
            break;

        default:
            Logger::error("Invalid IOP SIO2 register write at address 0x" + format("{:08X}", address));
            break;
    }
}