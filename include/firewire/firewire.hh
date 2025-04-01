#pragma once
#include <cstdint>
#include <cstring>
#include <log/log.hh>

/*
    Based on:
    LSI Logic ® 1394 Link Layer Controller Core - September 2001 | Rev. A
    LSI Logic ® 1394 Node Controller Core - August 2001 | Preliminary
    LSI Logic ® 1394 Physical Layer (PHY) Core - May 2001 | Complete?
    LSI Logic ® 1394 Lead Vehicle - September 2001 | Preliminary
*/

typedef enum
{
    PHY_Access = 0x014
} ILinkMMIO;

struct iLinkRegisters
{
    union {
        uint32_t raw;
        struct
        {
            uint32_t rxdat : 8;     // [7:0]   Receive PHY Register Data (RO) (PhyRxDat)
            uint32_t rx_addr : 4;   // [11:8]  Receive PHY Address (RO) (PhyRxAdr)
            uint32_t res1 : 4;      // [15:12] Reserved, must be 0
            uint32_t txdat : 8;     // [23:16] Write PHY Register Data (RW) (PhyRgDat)
            uint32_t reg_addr : 4;  // [27:24] PHY Access Address (RW) (PhyRgAdr)
            uint32_t res2 : 2;      // [29:28] Reserved, must be 0 (Res)
            uint32_t wr : 1;        // [30]    Write PHY (RW) (WrPhy)
            uint32_t rd : 1;        // [31]    Read PHY (RW) (RdPhy)
        };
    } phy_reg = {0};
};

struct PhyRegisters
{

};

class FireWire
{
  public:
    FireWire();

    iLinkRegisters ilink_regs;
    PhyRegisters phy_regs;

    uint32_t read(uint32_t address);
    void write(uint32_t address, uint32_t value);

  private:
};