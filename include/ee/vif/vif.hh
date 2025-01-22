#pragma once
#include <cstdint>

class VIF {
public:
    // Constants for register addresses
    static constexpr uint32_t VIF0 = 0x10003800;
    static constexpr uint32_t VIF0_ = 0x10003C00;
    static constexpr uint32_t VIF1 = 0x10003C00;
    static constexpr uint32_t VIF1_ = 0x10004000;

    static constexpr uint32_t VIF0_FIFO = 0x10004000;
    static constexpr uint32_t VIF1_FIFO = 0x10005000;

    static constexpr uint32_t VIF0_STAT = 0x10003800;
    static constexpr uint32_t VIF1_STAT = 0x10003C00;
    static constexpr uint32_t VIF0_FBRST = 0x10003810;
    static constexpr uint32_t VIF1_FBRST = 0x10003C10;
    static constexpr uint32_t VIF0_ERR = 0x10003820;
    static constexpr uint32_t VIF1_ERR = 0x10003C20;
    static constexpr uint32_t VIF0_MARK = 0x10003830;
    static constexpr uint32_t VIF1_MARK = 0x10003C30;
    static constexpr uint32_t VIF0_CYCLE = 0x10003840;
    static constexpr uint32_t VIF1_CYCLE = 0x10003C40;
    static constexpr uint32_t VIF0_MODE = 0x10003850;
    static constexpr uint32_t VIF1_MODE = 0x10003C50;
    static constexpr uint32_t VIF0_NUM = 0x10003860;
    static constexpr uint32_t VIF1_NUM = 0x10003C60;
    static constexpr uint32_t VIF0_MASK = 0x10003870;
    static constexpr uint32_t VIF1_MASK = 0x10003C70;
    static constexpr uint32_t VIF0_CODE = 0x10003880;
    static constexpr uint32_t VIF1_CODE = 0x10003C80;
    static constexpr uint32_t VIF0_ITOPS = 0x10003890;
    static constexpr uint32_t VIF1_ITOPS = 0x10003C90;
    static constexpr uint32_t VIF1_BASE = 0x10003CA0;
    static constexpr uint32_t VIF1_OFST = 0x10003CB0;
    static constexpr uint32_t VIF1_TOPS = 0x10003CC0;
    static constexpr uint32_t VIF0_ITOP = 0x100038D0;
    static constexpr uint32_t VIF1_ITOP = 0x10003CD0;
    static constexpr uint32_t VIF1_TOP = 0x10003CE0;
    static constexpr uint32_t VIF0_RN = 0x10003900;
    static constexpr uint32_t VIF1_RN = 0x10003D00;
    static constexpr uint32_t VIF0_CN = 0x10003940;
    static constexpr uint32_t VIF1_CN = 0x10003D40;

    VIF();

    // Methods to handle register reads and writes
    uint32_t read(uint32_t address);
    void write(uint32_t address, uint32_t value);

private:
    uint32_t vif0_stat;
    uint32_t vif1_stat;
    uint32_t vif0_fbrst;
    uint32_t vif1_fbrst;
    uint32_t vif0_err;
    uint32_t vif1_err;
    uint32_t vif0_mark;
    uint32_t vif1_mark;
    uint32_t vif0_cycle;
    uint32_t vif1_cycle;
    uint32_t vif0_mode;
    uint32_t vif1_mode;
    uint32_t vif0_num;
    uint32_t vif1_num;
    uint32_t vif0_mask;
    uint32_t vif1_mask;
    uint32_t vif0_code;
    uint32_t vif1_code;
    uint32_t vif0_itops;
    uint32_t vif1_itops;
    uint32_t vif1_base;
    uint32_t vif1_ofst;
    uint32_t vif1_tops;
    uint32_t vif0_itop;
    uint32_t vif1_itop;
    uint32_t vif1_top;
    uint32_t vif0_rn[4];
    uint32_t vif1_rn[4];
    uint32_t vif0_cn[4];
    uint32_t vif1_cn[4];
};
