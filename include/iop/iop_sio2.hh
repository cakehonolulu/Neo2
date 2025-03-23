#pragma once
#include <cstdint>
#include <cstring>
#include <log/log.hh>

class IOP_SIO2
{
  public:
    IOP_SIO2();

    uint32_t read(uint32_t address);
    void write(uint32_t address, uint32_t value);

  private:
    uint32_t send3[16]; // Command Parameters
    uint32_t send1[8];  // Port1 Control?
    uint32_t send2[8];  // Port2 Control?
    uint32_t fifoin;    // Data Write
    uint32_t fifoout;   // Data Read
    uint32_t ctrl;      // Control Register
    uint32_t recv1;     // Response Status 1
    uint32_t recv2;     // Response Status 2
    uint32_t recv3;     // Response Status 3
    uint32_t istat;     // Interrupt Flags
};