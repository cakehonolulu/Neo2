#pragma once

#include <ipu/ipu.hh>
#include <ee/vif/vif.hh>
#include <ee/dmac/dmac.hh>
#include <ee/timer/timer.hh>
#include <gif/gif.hh>
#include <gs/gs.hh>
#include <ee/intc/intc.hh>
#include <ee/sio/sio.hh>
#include <bus/tlb.hh>
#include <reg.hh>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include <bus/rdram.hh>
#include <iop/iop_timer.hh>
#include <iop/iop_dmac.hh>
#include <iop/iop_sio2.hh>
#include <sif/sif.hh>
#include <cdvd/cdvd.hh>
#include <firewire/firewire.hh>

enum class BusMode
{
    Ranged,
    SoftwareFastMem
};

const uint32_t PAGE_SIZE = 4 * 1024; // Page size = 4KB

class Bus
{
  private:
  public:
    Bus(BusMode mode = BusMode::SoftwareFastMem);

    std::vector<std::uint8_t> bios;
    std::vector<std::uint8_t> ram;
    std::vector<std::uint8_t> scratchpad;
    std::vector<std::uint8_t> iop_ram;

    TLB tlb;
    uintptr_t *address_space_r;
    uintptr_t *address_space_w;

    SIO sio;

    EE_INTC ee_intc;
    GIF gif;
    GS gs;
    EE_Timer timers;
    IOP_Timer iop_timers;
    EE_DMAC dmac;
    VIF vif;
    IPU ipu;
    RDRAM rdram;
    IOP_DMAC iop_dmac;
    IOP_SIO2 sio2;

    SIF sif;
    CDVD cdvd;
    FireWire firewire;

    std::uint32_t map_to_phys(std::uint32_t vaddr, const TLB &tlb);

    std::function<std::uint8_t(std::uint32_t)> read8;
    std::function<std::uint16_t(std::uint32_t)> read16;
    std::function<std::uint32_t(std::uint32_t)> read32;
    std::function<std::uint64_t(std::uint32_t)> read64;
    std::function<uint128_t(std::uint32_t)> read128;
    std::function<void(std::uint32_t, std::uint8_t)> write8;
    std::function<void(std::uint32_t, std::uint16_t)> write16;
    std::function<void(std::uint32_t, std::uint32_t)> write32;
    std::function<void(std::uint32_t, std::uint64_t)> write64;
    std::function<void(std::uint32_t, uint128_t)> write128;

    std::function<void(std::uint32_t, std::uint32_t, std::uint32_t)> write32_dbg;

    void load_bios(const std::string &bios_path);
    void fmem_init();
    std::uint8_t fmem_read8(uint32_t address);
    std::uint16_t fmem_read16(uint32_t address);
    std::uint32_t fmem_read32(uint32_t address);
    std::uint64_t fmem_read64(uint32_t address);
    uint128_t fmem_read128(uint32_t address);
    void fmem_write8(uint32_t address, uint8_t value);
    void fmem_write16(uint32_t address, uint16_t value);
    void fmem_write32(uint32_t address, uint32_t value);
    void fmem_write64(uint32_t address, uint64_t value);
    void fmem_write128(uint32_t address, uint128_t value);

    void fmem_write32_dbg(uint32_t address, uint32_t value, uint32_t pc);
};
