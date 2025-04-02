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
} i1934LLCCRegMap;

struct i1934LinkLayerControllerCoreRegisters
{
    /*
        PHY Access Register (0x014)

        +=======+=======+========+===============+=========================+=====================+==========+============+
        | 31    | 30    | 29  28 | 27         24 | 23                   16 | 15               12 | 11    8  | 7        0 |
        +=======+=======+========+===============+=========================+=====================+==========+============+
        | RdPhy | WrPhy | Res    | PhyRgAdr      | PhyRegDat               | Res                 | PhyRxAdr | PhyRxDat   |
        +-------+-------+--------+---------------+-------------------------+---------------------+----------+------------+
    */
    union {
        uint32_t value; // Full 32-bit value
        struct
        {
            uint32_t rxdat : 8;     // Bits     7-0: PhyRxDat (Receive PHY Register Data, RO)
            uint32_t rxadr : 4;     // Bits     11-8: PhyRxAAdr (Receive PHY Address, RO)
            uint32_t res1 : 4;      // Bits     15-12: Reserved
            uint32_t rgdat : 8;     // Bits     23-16: PhyRgDat (Write PHY Register Data, RW)
            uint32_t rgadr : 4;     // Bits     27-24: PhyRgAdr (PHY Access Address, RW)
            uint32_t res2 : 2;      // Bits     29-28: Reserved
            uint32_t wrphy : 1;     // Bit      30: WrPhy (Write PHY, RW)
            uint32_t rdphy : 1;     // Bit      31: RdPhy (Read PHY, RW)
        };
    } phy_access_reg = {0};
};

struct i1394PhysicalLayerCoreRegisters
{
    union {
        uint8_t value; // Full 8-bit value
        struct
        {
            uint8_t physical_id : 6;    // Bits     5-0: Physical_ID (Physical ID, RO)
            uint8_t r : 1;              // Bit        6: R (Root, RO)
            uint8_t ps : 1;             // Bit        7: PS (Power Status, RO)
        };
    } reg_00 = {0};

    union {
        uint8_t value; // Full 8-bit value
        struct
        {
            uint32_t rhb : 1;           // Bit        0: RHB (Root Hold-Off Bit, RW)
            uint32_t ibr : 1;           // Bit        1: IBR (Initiate Bus Reset, RW)
            uint32_t gap_count : 6;     // Bits     7-2: Gap_Count (Gap Count, RW)
        };
    } reg_01 = {0xFC};
};

class FireWire
{
  public:
    FireWire();

    i1934LinkLayerControllerCoreRegisters llc_core_regs;
    i1394PhysicalLayerCoreRegisters pl_core_regs;

    uint32_t read(uint32_t address);
    void write(uint32_t address, uint32_t value);

  private:
};