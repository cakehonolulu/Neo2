#pragma once
#include <cstdint>

typedef enum
{
    SRD     = 0b0000,
    SWR     = 0b0001,
    SETR    = 0b0010,
    SETF    = 0b0100,
    CLRR    = 0b1011
} RDRAMCommand;

typedef enum
{
    INIT    = 0x021,
    TEST34  = 0x022,
    NAPX    = 0x045,
    DEVID   = 0x040,
    CCA     = 0x043,
    CCB     = 0x044,
    PDNXA   = 0x046,
    PDNX    = 0x047,
    TPARM   = 0x048,
    TFRM    = 0x049,
    TCDLY1  = 0x04A,
    SKIP    = 0x04B,
    TCYCLE  = 0x04C,
    TEST77  = 0x04D,
    TEST78  = 0x04E
} RDRAMRegister;

struct RDRAM_IC
{
    // INIT Control Register (R/W)
    union {
        uint16_t value; // Full 14-bit register value
        struct
        {
            /* Serial Device ID, Device address for control register read/write */
            uint32_t sdevid : 6;    // Bits   0-5: SDEVID[0:5]
            /* Power Select exit. PDN/NAP exit with device address on DQA5...O. */
            uint32_t psx    : 1;    // Bit      6: PSX
            /* SIO Repeater. Used to initialize RDRAM. */
            uint32_t srp    : 1;    // Bit      7: SRP
            /* NAP Self-Refresh. Enables Self-Refresh in NAP Mode. */
            uint32_t nsr    : 1;    // Bit      8: NSR
            /* PDN Self-Refresh. Enables Self-Refresh in PDN Mode. */
            uint32_t psr    : 1;    // Bit     10: PSR
            /* Low-power Self-Refresh. Enables low-power Self-Refresh. */
            uint32_t lsr    : 1;    // Bit     11: LSR
            /* Temperature sensing enable. */
            uint32_t ten    : 1;    // Bit     12: TEN
            /* Temperature sensing output. */
            uint32_t tsq    : 1;    // Bit     13: TSQ
            /* RDRAM disable. */
            uint32_t dis    : 1;    // Bit     14: DIS
        };
    } init = {0};

    // TEST34 Control Register (R/W)
    /* Test register. Do not read or write after SIO reset. */
    uint16_t test34 = 0x0;

    // CNFGA Control Register (R/O)
    union {
        uint16_t value; // Full 16-bit register value
        struct
        {
            /* Refresh bank bits. Used for Multi-Bank refresh. */
            uint32_t refbit : 3;    // Bits   0-2: REFBIT[0:2]
            /* Double. Specifies Doubled-Bank architecture. */
            uint32_t dbl    : 1;    // Bit      3: DBL
            /* Manufacturer version. Manufacturer identification number. */
            uint32_t mver   : 6;    // Bits   4-9: MVER[4:9]
            /* Protocol version. Specifies version of Direct protocol supported. */
            uint32_t pver   : 6;    // Bits 10-15: NSR[10:15]
        };
    } cnfga = {0};

    // CNFGB Control Register (R/O)
    union {
        uint16_t value; // Full 16-bit register value
        struct
        {
            /* Byte. Specifies an 8-bit or 9-bit byte size. */
            uint32_t byt    : 1;    // Bit        0: BYT
            /* Device type. Device can be RDRAM or some other device category. */
            uint32_t devtyp : 3;    // Bits     1-3: DBL
            /* Split-core. Each core half is an individual dependent core. */
            uint32_t spt    : 1;    // Bit        4: MVER
            /* Core organization. Bank, row, column address field sizes. */
            uint32_t corg   : 5;    // Bits     5-9: CORG[5:9]
            /* Stepping version. Mask version number. */
            uint32_t sver   : 6;    // Bits   10-15: SVER[10:15]
        };
    } cnfgb = {0};

    // DEVID Control Register (5-bits) (R/W)
    /* Device ID. Device address for memory read/write. */
    uint8_t devid = 0x0;

    // REFB Control Register (5-bits) (R/W)
    /* Refresh bank. Next bank to be refreshed by Self-Refresh. */
    uint8_t refb = 0x0;

    // REFR Control Register (9-bits) (R/W)
    /* Refresh row. Next row to be refreshed by REFA, Self-Refresh. */
    uint16_t refr = 0x0;

    // CCA Control Register (R/W)
    union {
        uint8_t value; // Full 8-bit register value
        struct
        {
            /* Current control A. Control |OL output current for DQA. */
            uint32_t cca    : 7; // Bits       0-7: CCA[0:7]
            /* Asymmetry control. Controls asymmetry of VOL/VOH swing for DQA. */
            uint32_t asyma  : 1; // Bit          8: ASYMA
        };
    } cca = {0};

    // CCB Control Register (R/W)
    union {
        uint8_t value; // Full 8-bit register value
        struct
        {
            /* Current control B. Control |OL output current for DQB. */
            uint32_t ccb    : 7; // Bits       0-7: CCB[0:7]
            /* Asymmetry control. Controls asymmetry of VOL/VOH swing for DQB. */
            uint32_t asymb  : 1; // Bit          8: ASYMB
        };
    } ccb = {0};

    // NAPX Control Register (R/W)
    union {
        uint16_t value; // Full 11-bit register value
        struct
        {
            /* NAP exit. Specifies length of NAP exit phase A. */
            uint32_t napxa  : 5;    // Bits       0-4: NAPXA[0:4]
            /* NAP exit. Specifies length of NAP exit phaseA + phase B. */
            uint32_t napx   : 5;    // Bits       5-9: NAPX[5:9]
            /* DQ select. Selects CMD framing for NAP/PDN exit. */
            uint32_t dqs    : 1;    // Bit         10: MVER
        };
    } napx = {0};

    // PDNXA Control Register (6-bits) (R/W)
    /* PDN exit. Specifies length of PDN exit phase A. */
    uint8_t pdnxa = 0x0;

    // PDNX Control Register (3-bits) (R/W)
    /* PDN exit. Specifies length of PDN exit phaseA + phase B. */
    uint16_t pdnx = 0x0;

    // TPARM Control Register (R/W)
    union {
        uint8_t value; // Full 7-bit register value
        struct
        {
            /* tCAS-C core parameter. Determines tOFFP datasheet parameter. */
            uint32_t tcas   : 2; // Bits       0-1: TCAS[0:1]
            /* tCLS-C core parameter. Determines tCAC and tOFFP datasheet parameter. */
            uint32_t tcls   : 2; // Bits       2-3: TCLS[0:1]
            /* tCDLY0-C core parameter. Programmable delay for read data. */
            uint32_t tcdly0 : 3; // Bit        4-6: TCDLY0[0:2]
        };
    } tparm = {0};

    // TFRM Control Register (4-bits) (R/W)
    /* tFRM-C core parameter. Determines ROW-to-COL packet framing interval. */
    uint8_t tfrm = 0x0;
    
    // TCDLY1 Control Register (2-bits) (R/W)
    /* tCDLY1-C datasheet parameter. Programmable delay for read data. */
    uint8_t tcdly1 = 0x0;

    // TCYCLE Control Register (6-bits) (R/W)
    /* tCYCLE datasheet parameter. Specifies cycle time in 64 ps units. */
    uint8_t tcycle = 0x0;

    // SKIP Control Register (R/W & R/O)
    union {
        uint8_t value; // Full 3-bit register value
        struct
        {
            /* Autoskip value established by the SETF command. */
            /* R/O */
            uint32_t as     : 1; // Bit       0: AS
            /* Manual skip enable. Allows the MS value to override the AS value. */
            /* R/W */
            uint32_t mse    : 1; // Bit       1: MSE
            /* Manual skip value. */
            /* R/W */
            uint32_t ms     : 1; // Bit       2: MS
        };
    } skip = {0};

    /* Test register. Write with zero after SIO reset. */
    uint16_t test77 = 0x0;

    /* Test register. Do not read or write after SIO reset. */
    uint16_t test78 = 0x0;

    /* Test register. Do not read or write after SIO reset. */
    uint16_t test79 = 0x0;
};

class RDRAM
{
  public:
    // Constants for register addresses
    static constexpr uint32_t MCH_RICM = 0x1000F430;
    static constexpr uint32_t MCH_DRD = 0x1000F440;

    // RDRAM Initialization Control Register
    union {
        uint32_t value; // Full 32-bit register value
        struct
        {
            uint32_t sdev_low   : 5;    // Bits   0-4: SDEV[0:4]
            uint32_t sbc        : 1;    // Bit      5: SBC (broadcast flag)
            uint32_t sop        : 4;    // Bits   6-9: SOP (serial opcode)
            uint32_t sdev_high  : 1;    // Bit     10: SDEV[5]
            uint32_t reserved1  : 5;    // Bits 11-15: Reserved
            uint32_t sa         : 12;   // Bits 16-27: SA (serial address)
            uint32_t reserved2  : 3;    // Bits 28-30: Reserved
            uint32_t busy       : 1;    // Bit     31: Busy
        };
    } mch_ricm_ = {0};
    
    // RDRAM Device Register Data Register
    uint32_t mch_drd_ = 0x0;

    /*
    * Apparently all PS2's came generally with 2 RDRAM ICs.
    * Let's honour that.
    * 
    * https://www.psdevwiki.com/ps2/Rambus_DRAM
    * 
    * TODO: Search how many RDRAM ICs exist on DESRs and,
    * whenever we have a BIOS ident. system with hashes,
    * change the number of RDRAM ICs accordingly.
    */
    RDRAM_IC rdram_ic[2];

    RDRAM();

    // Methods to handle register reads and writes
    uint32_t read(uint32_t address);
    void write(uint32_t address, uint32_t value);
    void cmd();
    void srd();
    void swr();
    void setr();
};
