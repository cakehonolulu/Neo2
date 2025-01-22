#pragma once
#include <cstdint>

class EE_INTC {
public:
    // Constants for register addresses
    static constexpr uint32_t INTC_STAT_ADDR = 0x1000F000;
    static constexpr uint32_t INTC_MASK_ADDR = 0x1000F010;

    EE_INTC();

    // Methods to handle register reads and writes
    uint32_t read(uint32_t address);
    void write(uint32_t address, uint32_t value);

    // Interrupt handling methods
    void request_interrupt(uint32_t irq);   // Raise an IRQ
    void acknowledge_interrupt(uint32_t irq); // Acknowledge an IRQ

    bool is_interrupt_pending() const;  // Check if INT0 should be asserted

private:
    uint32_t intc_stat; // Interrupt status register
    uint32_t intc_mask; // Interrupt mask register
};
