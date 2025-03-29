#pragma once
#include <cstdint>
#include <cstring>
#include <log/log.hh>

class CDVD
{
  public:
    CDVD();

    uint32_t read(uint32_t address);
    void write(uint32_t address, uint32_t value);

  private:
    uint32_t current_n_command; // Current N command
    uint32_t n_command_status;  // N command status
    uint32_t n_command_param;   // N command param
    uint32_t cdvd_error;        // CDVD error
    uint32_t i_stat;            // CDVD I_STAT
    uint32_t drive_status;      // CDVD drive status
    uint32_t sticky_drive_status; // Sticky drive status
    uint32_t disk_type;         // CDVD disk type
    uint32_t current_s_command; // Current S command
    uint32_t s_command_status;  // S command status
    uint32_t s_command_param;   // S command params
    uint32_t s_command_result;  // S command result
};