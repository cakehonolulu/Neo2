#pragma once
#include <cstdint>

#include <log/log.hh>

class IOP_DMAC
{
  public:
    IOP_DMAC();

    uint32_t read(uint32_t address);
    void write(uint32_t address, uint32_t value);

  private:
    struct DMA_Channel
    {
        uint32_t madr; // Start address of transfer in RAM
        uint32_t bcr;  // Block size and count
        uint32_t chcr; // Control register
        uint32_t tadr; // Tag address
    };

    DMA_Channel channels[13]; // 13 DMA channels

    uint32_t dpcr;      // DMA Priority/Enable
    uint32_t dpcr2;     // DMA Priority/Enable 2
    uint32_t dicr;      // DMA Interrupt Register
    uint32_t dicr2;     // DMA Interrupt Register 2
    uint32_t dmace;     // Global DMA Enable
    uint32_t dmacinten; // Global DMA Interrupt Control
};