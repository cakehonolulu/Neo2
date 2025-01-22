#include <ee/vif/vif.hh>
#include <log/log.hh>
#include <format>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

VIF::VIF()
    : vif0_stat(0),
      vif1_stat(0),
      vif0_fbrst(0),
      vif1_fbrst(0),
      vif0_err(0),
      vif1_err(0),
      vif0_mark(0),
      vif1_mark(0),
      vif0_cycle(0),
      vif1_cycle(0),
      vif0_mode(0),
      vif1_mode(0),
      vif0_num(0),
      vif1_num(0),
      vif0_mask(0),
      vif1_mask(0),
      vif0_code(0),
      vif1_code(0),
      vif0_itops(0),
      vif1_itops(0),
      vif1_base(0),
      vif1_ofst(0),
      vif1_tops(0),
      vif0_itop(0),
      vif1_itop(0),
      vif1_top(0) {
    for (int i = 0; i < 4; ++i) {
        vif0_rn[i] = 0;
        vif1_rn[i] = 0;
        vif0_cn[i] = 0;
        vif1_cn[i] = 0;
    }
    Logger::set_subsystem("VIF");
}

uint32_t VIF::read(uint32_t address) {
    std::string reg_name;
    uint32_t value = 0;

    if (address >= VIF0 && address < VIF0_) {
        if (address == VIF0_STAT) {
            reg_name = "VIF0_STAT";
            value = vif0_stat;
        } else if (address == VIF0_FBRST) {
            reg_name = "VIF0_FBRST";
            value = vif0_fbrst;
        } else if (address == VIF0_ERR) {
            reg_name = "VIF0_ERR";
            value = vif0_err;
        } else if (address == VIF0_MARK) {
            reg_name = "VIF0_MARK";
            value = vif0_mark;
        } else if (address == VIF0_CYCLE) {
            reg_name = "VIF0_CYCLE";
            value = vif0_cycle;
        } else if (address == VIF0_MODE) {
            reg_name = "VIF0_MODE";
            value = vif0_mode;
        } else if (address == VIF0_NUM) {
            reg_name = "VIF0_NUM";
            value = vif0_num;
        } else if (address == VIF0_MASK) {
            reg_name = "VIF0_MASK";
            value = vif0_mask;
        } else if (address == VIF0_CODE) {
            reg_name = "VIF0_CODE";
            value = vif0_code;
        } else if (address == VIF0_ITOPS) {
            reg_name = "VIF0_ITOPS";
            value = vif0_itops;
        } else if (address == VIF0_ITOP) {
            reg_name = "VIF0_ITOP";
            value = vif0_itop;
        } else if (address >= VIF0_RN && address < VIF0_RN + 16) {
            reg_name = format("VIF0_RN+{}", (address - VIF0_RN) / 4);
            value = vif0_rn[(address - VIF0_RN) / 4];
        } else if (address >= VIF0_CN && address < VIF0_CN + 16) {
            reg_name = format("VIF0_CN+{}", (address - VIF0_CN) / 4);
            value = vif0_cn[(address - VIF0_CN) / 4];
        } else {
            Logger::error("Invalid VIF0 register read at address 0x" + format("{:08X}", address));
            return 0;
        }
    } else if (address >= VIF1 && address < VIF1_) {
        if (address == VIF1_STAT) {
            reg_name = "VIF1_STAT";
            value = vif1_stat;
        } else if (address == VIF1_FBRST) {
            reg_name = "VIF1_FBRST";
            value = vif1_fbrst;
        } else if (address == VIF1_ERR) {
            reg_name = "VIF1_ERR";
            value = vif1_err;
        } else if (address == VIF1_MARK) {
            reg_name = "VIF1_MARK";
            value = vif1_mark;
        } else if (address == VIF1_CYCLE) {
            reg_name = "VIF1_CYCLE";
            value = vif1_cycle;
        } else if (address == VIF1_MODE) {
            reg_name = "VIF1_MODE";
            value = vif1_mode;
        } else if (address == VIF1_NUM) {
            reg_name = "VIF1_NUM";
            value = vif1_num;
        } else if (address == VIF1_MASK) {
            reg_name = "VIF1_MASK";
            value = vif1_mask;
        } else if (address == VIF1_CODE) {
            reg_name = "VIF1_CODE";
            value = vif1_code;
        } else if (address == VIF1_ITOPS) {
            reg_name = "VIF1_ITOPS";
            value = vif1_itops;
        } else if (address == VIF1_BASE) {
            reg_name = "VIF1_BASE";
            value = vif1_base;
        } else if (address == VIF1_OFST) {
            reg_name = "VIF1_OFST";
            value = vif1_ofst;
        } else if (address == VIF1_TOPS) {
            reg_name = "VIF1_TOPS";
            value = vif1_tops;
        } else if (address == VIF1_ITOP) {
            reg_name = "VIF1_ITOP";
            value = vif1_itop;
        } else if (address == VIF1_TOP) {
            reg_name = "VIF1_TOP";
            value = vif1_top;
        } else if (address >= VIF1_RN && address < VIF1_RN + 16) {
            reg_name = format("VIF1_RN+{}", (address - VIF1_RN) / 4);
            value = vif1_rn[(address - VIF1_RN) / 4];
        } else if (address >= VIF1_CN && address < VIF1_CN + 16) {
            reg_name = format("VIF1_CN+{}", (address - VIF1_CN) / 4);
            value = vif1_cn[(address - VIF1_CN) / 4];
        } else {
            Logger::error("Invalid VIF1 register read at address 0x" + format("{:08X}", address));
            return 0;
        }
    } else if (address >= VIF0_FIFO && address < (VIF0_FIFO + 0x1000)) {
        reg_name = format("VIF0_FIFO+{}", (address - VIF0_FIFO) / 4);
        value = 0; // Placeholder
    } else if (address >= VIF1_FIFO && address < (VIF1_FIFO + 0x1000)) {
        reg_name = format("VIF1_FIFO+{}", (address - VIF1_FIFO) / 4);
        value = 0; // Placeholder
    } else {
        Logger::error("Invalid VIF register read at address 0x" + format("{:08X}", address));
        return 0;
    }

    Logger::info("VIF register read from " + reg_name + " with value 0x" + format("{:08X}", value));
    return value;
}

void VIF::write(uint32_t address, uint32_t value) {
    std::string reg_name;

    if (address >= VIF0 && address < VIF0_) {
        if (address == VIF0_STAT) {
            reg_name = "VIF0_STAT";
            vif0_stat = value;
        } else if (address == VIF0_FBRST) {
            reg_name = "VIF0_FBRST";
            vif0_fbrst = value;
        } else if (address == VIF0_ERR) {
            reg_name = "VIF0_ERR";
            vif0_err = value;
        } else if (address == VIF0_MARK) {
            reg_name = "VIF0_MARK";
            vif0_mark = value;
        } else if (address == VIF0_CYCLE) {
            reg_name = "VIF0_CYCLE";
            vif0_cycle = value;
        } else if (address == VIF0_MODE) {
            reg_name = "VIF0_MODE";
            vif0_mode = value;
        } else if (address == VIF0_NUM) {
            reg_name = "VIF0_NUM";
            vif0_num = value;
        } else if (address == VIF0_MASK) {
            reg_name = "VIF0_MASK";
            vif0_mask = value;
        } else if (address == VIF0_CODE) {
            reg_name = "VIF0_CODE";
            vif0_code = value;
        } else if (address == VIF0_ITOPS) {
            reg_name = "VIF0_ITOPS";
            vif0_itops = value;
        } else if (address == VIF0_ITOP) {
            reg_name = "VIF0_ITOP";
            vif0_itop = value;
        } else if (address >= VIF0_RN && address < VIF0_RN + 16) {
            reg_name = format("VIF0_RN+{}", (address - VIF0_RN) / 4);
            vif0_rn[(address - VIF0_RN) / 4] = value;
        } else if (address >= VIF0_CN && address < VIF0_CN + 16) {
            reg_name = format("VIF0_CN+{}", (address - VIF0_CN) / 4);
            vif0_cn[(address - VIF0_CN) / 4] = value;
        } else {
            Logger::error("Invalid VIF0 register write at address 0x" + format("{:08X}", address));
            return;
        }
    } else if (address >= VIF1 && address < VIF1_) {
        if (address == VIF1_STAT) {
            reg_name = "VIF1_STAT";
            vif1_stat = value;
        } else if (address == VIF1_FBRST) {
            reg_name = "VIF1_FBRST";
            vif1_fbrst = value;
        } else if (address == VIF1_ERR) {
            reg_name = "VIF1_ERR";
            vif1_err = value;
        } else if (address == VIF1_MARK) {
            reg_name = "VIF1_MARK";
            vif1_mark = value;
        } else if (address == VIF1_CYCLE) {
            reg_name = "VIF1_CYCLE";
            vif1_cycle = value;
        } else if (address == VIF1_MODE) {
            reg_name = "VIF1_MODE";
            vif1_mode = value;
        } else if (address == VIF1_NUM) {
            reg_name = "VIF1_NUM";
            vif1_num = value;
        } else if (address == VIF1_MASK) {
            reg_name = "VIF1_MASK";
            vif1_mask = value;
        } else if (address == VIF1_CODE) {
            reg_name = "VIF1_CODE";
            vif1_code = value;
        } else if (address == VIF1_ITOPS) {
            reg_name = "VIF1_ITOPS";
            vif1_itops = value;
        } else if (address == VIF1_BASE) {
            reg_name = "VIF1_BASE";
            vif1_base = value;
        } else if (address == VIF1_OFST) {
            reg_name = "VIF1_OFST";
            vif1_ofst = value;
        } else if (address == VIF1_TOPS) {
            reg_name = "VIF1_TOPS";
            vif1_tops = value;
        } else if (address == VIF1_ITOP) {
            reg_name = "VIF1_ITOP";
            vif1_itop = value;
        } else if (address == VIF1_TOP) {
            reg_name = "VIF1_TOP";
            vif1_top = value;
        } else if (address >= VIF1_RN && address < VIF1_RN + 16) {
            reg_name = format("VIF1_RN+{}", (address - VIF1_RN) / 4);
            vif1_rn[(address - VIF1_RN) / 4] = value;
        } else if (address >= VIF1_CN && address < VIF1_CN + 16) {
            reg_name = format("VIF1_CN+{}", (address - VIF1_CN) / 4);
            vif1_cn[(address - VIF1_CN) / 4] = value;
        } else {
            Logger::error("Invalid VIF1 register write at address 0x" + format("{:08X}", address));
            return;
        }
    } else if (address >= VIF0_FIFO && address < (VIF0_FIFO + 0x1000)) {
        reg_name = format("VIF0_FIFO+{}", (address - VIF0_FIFO) / 4);
        // Handle FIFO write
    } else if (address >= VIF1_FIFO && address < (VIF1_FIFO + 0x1000)) {
        reg_name = format("VIF1_FIFO+{}", (address - VIF1_FIFO) / 4);
        // Handle FIFO write
    } else {
        Logger::error("Invalid VIF register write at address 0x" + format("{:08X}", address));
        return;
    }

    Logger::info("VIF register written to " + reg_name + " with value 0x" + format("{:08X}", value));
}
