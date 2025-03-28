#pragma once
#include <cstdint>
#include <cstring>
#include <log/log.hh>

class SIF
{
  public:
    SIF();

    uint32_t read(uint32_t address);
    void write(uint32_t address, uint32_t value);

  private:
    uint32_t mscom; // SIF_MSCOM
    uint32_t smcom; // SIF_SMCOM
    uint32_t msflg; // SIF_MSFLG
    uint32_t smflg; // SIF_SMFLG
    uint32_t ctrl;  // SIF_CTRL
    uint32_t bd6;   // SIF_BD6
};