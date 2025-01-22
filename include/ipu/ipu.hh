#include <cstdint>

class IPU {
public:
    IPU();

    void write(uint32_t address, uint64_t value);
    uint64_t read(uint32_t address);

private:
    uint64_t ipu_cmd;
    uint32_t ipu_ctrl;
    uint32_t ipu_bp;

    // Define register addresses
    static constexpr uint32_t IPU_CMD = 0x10002000;
    static constexpr uint32_t IPU_CTRL = 0x10002010;
    static constexpr uint32_t IPU_BP = 0x10002020;
    static constexpr uint32_t IPU_FIFO_OUT = 0x10007000;
    static constexpr uint32_t IPU_FIFO_IN = 0x10007010;
};
