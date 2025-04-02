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
    NodeID              = 0x000,
    PHY_Access          = 0x014,
    Interrupt0          = 0x020,
    Interrupt1          = 0x028,
    Interrupt2          = 0x030,
    UBUF_Receive_Clear  = 0x04C
} i1934LLCCRegMap;

struct i1934LinkLayerControllerCoreRegisters
{
    /*
        Control and Status Registers (CSRs)
    */
    /*
        Node ID Register (0x000)

        +================+============+====================+
        | 31          22 | 21      16 | 15               1 | 0
        +================+============+====================+
        | BusID          | Offset ID  | Res                | Valid
        +----------------+------------+--------------------+
    */
    union {
        uint32_t value; // Full 32-bit value
        struct
        {
            uint32_t valid : 1;     // Bit          0: Valid (Valid)
            uint32_t res : 15;      // Bits      15-1: Reserver (Reserved)
            uint32_t offsetid : 6;  // Bits     21-16: OffsetID (Offset ID, RO)
            uint32_t busid : 10;    // Bits     31-22: BusID (Bus ID, RW)
        };
    } node_id_reg = {0x03FF0000};

    /*
        1394 Registers
    */
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

    /*
        Interrupt 0 Register (0x020)

        +========+========+========+===========+========+===========+========+=======+===========+=====+=======+========+=======+=======+========+=========+
        | 31     | 30     | 29     | 28        | 27     | 26        | 25     | 24    | 23        | 22  | 21    | 20     | 19    | 18    | 17     | 16      |
        +========+========+========+===========+========+===========+========+=======+===========+=====+=======+========+=======+=======+========+=========+
        | PhyInt | PhyRRx | PhyRst | ArbRstGap | CmdRst | SntBsyAck | HdrErr | TCErr | SubActGap | URx | CycTL | CycSec | CycSt | CycDn | CycPnd | CycLost |
        +--------+--------+--------+-----------+--------+-----------+--------+-------+-----------+-----+-------+--------+-------+-------+--------+---------+

        +==========+=========+=========+========+=======+=====+========+=======+======+=======+=======+=======+=====+=======+======+======+
        | 15       | 14      | 13      | 12     | 11    | 10  | 9      | 8     | 7    | 6     | 5     | 4     | 3   | 2     | 1    | 0    |
        +==========+=========+=========+========+=======+=====+========+=======+======+=======+=======+=======+=====+=======+======+======+
        | CycArbFl | AckRcvd | AckMiss | InvAck | RetEx | STO | PBCntR | UResp | FmtE | TxStk | SYTTT | SYTTR | Res | CIPHE | DRFO | DRFR |
        +----------+---------+---------+--------+-------+-----+--------+-------+------+-------+-------+-------+-----+-------+------+------+
    */
    union {
        uint32_t value; // Full 32-bit value
        struct
        {
            uint32_t drfr : 1;      // Bit      0: DRFR (DBUF Receive FIFO Reception)
            uint32_t drfo : 1;      // Bit      1: DRFO (DBUF Receive FIFO Overflow)
            uint32_t ciphe : 1;     // Bit      2: CIPHE (CIP Header Error)
            uint32_t reserved : 1;  // Bit      3: Reserved
            uint32_t syttr : 1;     // Bit      4: SYT Time Stamp Received
            uint32_t syttt : 1;     // Bit      5: SYT Time Stamp Transmitted
            uint32_t txstk : 1;     // Bit      6: Tx Stuck
            uint32_t fmte : 1;      // Bit      7: Format Error
            uint32_t uresp : 1;     // Bit      8: Unexpected Response
            uint32_t pbcntr : 1;    // Bit      9: Packet/Byte Count Reached
            uint32_t sto : 1;       // Bit     10: Split Time Out
            uint32_t retex : 1;     // Bit     11: Retry Attempts Exhausted
            uint32_t invack : 1;    // Bit     12: Invalid Acknowledge
            uint32_t ackmiss : 1;   // Bit     13: Acknowledge Missing
            uint32_t ackrcvd : 1;   // Bit     14: Acknowledge Received
            uint32_t cycarbfl : 1;  // Bit     15: Cycle Arbitration Failed
            uint32_t cyclost : 1;   // Bit     16: Cycle Lost
            uint32_t cycpnd : 1;    // Bit     17: Cycle Pending
            uint32_t cycdn : 1;     // Bit     18: Cycle Done
            uint32_t cycst : 1;     // Bit     19: Cycle Start
            uint32_t cycsec : 1;    // Bit     20: Cycle Second
            uint32_t cyctl : 1;     // Bit     21: Cycle Too Long
            uint32_t urx : 1;       // Bit     22: UBUF Packet Received
            uint32_t subactgap : 1; // Bit     23: Subaction Gap
            uint32_t tcerr : 1;     // Bit     24: Transaction Code Error
            uint32_t hdrerr : 1;    // Bit     25: Header Error
            uint32_t sntbsyack : 1; // Bit     26: Sent Busy/Tardy Acknowledge
            uint32_t cmdrst : 1;    // Bit     27: Command Reset
            uint32_t arbrstgap : 1; // Bit     28: Arbitration Reset Gap
            uint32_t phyrst : 1;    // Bit     29: PHY Reset
            uint32_t phyrrx : 1;    // Bit     30: PHY Register Received
            uint32_t phyint : 1;    // Bit     31: PHY Interrupt
        };
    } interrupt0 = {0};

    /*
        Interrupt 1 Register (0x028)

        +===================================+=====+======+
        | 31                              2 | 1   | 0    |
        +===================================+=====+======+
        | Res                               | UTD | DTFO |
        +-----------------------------------+-----+------+
    */
    union {
        uint32_t value; // Full 32-bit value
        struct
        {
            uint32_t dtfo : 1;      // Bit 0: DTFO (DBUF Transmit FIFO Overflow)
            uint32_t utd : 1;       // Bit 1: UTD (UBUF Transmission Done)
            uint32_t reserved : 30; // Bits 2-31: Reserved (must be written as zeros)
        };
    } interrupt1 = {0};

    /*
        Interrupt 2 Register (0x030)

        +=====================================+========+
        | 31                                1 | 0      |
        +=====================================+========+
        | Res                                 | LinkOn |
        +-------------------------------------+--------+
    */
    union {
        uint32_t value; // Full 32-bit value
        struct
        {
            uint32_t linkon : 1;    // Bit 0: LinkOn (Link On)
            uint32_t reserved : 31; // Bits 1-31: Reserved (must be written as zeros)
        };
    } interrupt2 = {0};

    /*
        UBUF Receive Clear (0x04C)
    */
    uint32_t ubuf_recv_clear = 0;
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
            uint8_t rhb : 1;            // Bit        0: RHB (Root Hold-Off Bit, RW)
            uint8_t ibr : 1;            // Bit        1: IBR (Initiate Bus Reset, RW)
            uint8_t gap_count : 6;      // Bits     7-2: Gap_Count (Gap Count, RW)
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