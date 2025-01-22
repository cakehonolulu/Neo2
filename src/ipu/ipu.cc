#include <ipu/ipu.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

IPU::IPU()
    : ipu_cmd(0),
      ipu_ctrl(0),
      ipu_bp(0) {
    Logger::set_subsystem("IPU");
}

void IPU::write(uint32_t address, uint64_t value) {
    std::string reg_name;

    if (address == IPU_CMD) {
        reg_name = "IPU_CMD";
        ipu_cmd = value;
    } else if (address == IPU_CTRL) {
        reg_name = "IPU_CTRL";
        ipu_ctrl = static_cast<uint32_t>(value);
    } else if (address == IPU_BP) {
        reg_name = "IPU_BP";
        ipu_bp = static_cast<uint32_t>(value);
    } else if (address == IPU_BP) {
        reg_name = "IPU_BP";
        ipu_bp = static_cast<uint32_t>(value);
    } else if ((address >= IPU_FIFO_OUT) && (address < (IPU_FIFO_OUT + 0x10))) {
        reg_name = "IPU_FIFO_OUT";
        // ?
    } else if ((address >= IPU_FIFO_IN) && (address < (IPU_FIFO_IN + 0x10))) {
        reg_name = "IPU_FIFO_IN";
        // ?
    } else {
        Logger::error("Invalid IPU register write at address 0x" + format("{:08X}", address));
        return;
    }

    Logger::info("IPU register written to " + reg_name + " with value 0x" + format("{:016X}", value));
}

uint64_t IPU::read(uint32_t address) {
    std::string reg_name;
    uint64_t value = 0;

    if (address == IPU_CMD) {
        reg_name = "IPU_CMD";
        value = ipu_cmd;
    } else if (address == IPU_CTRL) {
        reg_name = "IPU_CTRL";
        value = ipu_ctrl;
    } else if (address == IPU_BP) {
        reg_name = "IPU_BP";
        value = ipu_bp;
    } else if ((address >= IPU_FIFO_OUT) && (address < (IPU_FIFO_OUT + 0x10))) {
        reg_name = "IPU_FIFO_OUT";
        // ?
    } else if ((address >= IPU_FIFO_IN) && (address < (IPU_FIFO_IN + 0x10))) {
        reg_name = "IPU_FIFO_IN";
        // ?
    } else {
        Logger::error("Invalid IPU register read at address 0x" + format("{:08X}", address));
        return 0;
    }

    Logger::info("IPU register read from " + reg_name + " with value 0x" + format("{:016X}", value));
    return value;
}
