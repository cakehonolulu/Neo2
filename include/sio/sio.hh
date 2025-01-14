#pragma once

#include <cstdint>
#include <string>

// SIO Registers
struct SIORegisters {
    uint32_t LCR;  // Line control register
    uint32_t LSR;  // Line status register
    uint32_t IER;  // Interrupt enable register
    uint32_t ISR;  // Interrupt status register
    uint32_t FCR;  // FIFO control register
    uint32_t BGR;  // Baud rate control register
    uint32_t TXFIFO; // Transmit FIFO register
    uint32_t RXFIFO; // Receive FIFO register
};

class SIO {
public:
    static SIORegisters sio;

    static void write(uint32_t address, uint32_t value);
    static uint32_t read(uint32_t address);

    static std::string ee_tx_buffer;
};
