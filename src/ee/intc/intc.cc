#include <ee/intc/intc.hh>
#include <log/log.hh>
#include <neo2.hh>

EE_INTC::EE_INTC()
    : intc_stat(0),    // No IRQs raised initially
      intc_mask(0xFFFFFFFF) {
    Logger::set_subsystem("INTC");
} // All IRQs masked by default

uint32_t EE_INTC::read(uint32_t address) {
    std::string reg_name;
    switch (address) {
        case INTC_STAT_ADDR:
            reg_name = "INTC_STAT";
            break;
        case INTC_MASK_ADDR:
            reg_name = "INTC_MASK";
            break;
        default:
            Logger::error("Invalid INTC register read at address 0x" + format("{:08X}", address));
            Neo2::exit(1, Neo2::Subsystem::EE_INTC);
    }
    Logger::info("INTC register read from " + reg_name);
    return (address == INTC_STAT_ADDR) ? intc_stat : intc_mask;
}

void EE_INTC::write(uint32_t address, uint32_t value) {
    std::string reg_name;
    switch (address) {
        case INTC_STAT_ADDR:
            reg_name = "INTC_STAT";
            intc_stat &= ~value;  // Writing '1' to a bit acknowledges (clears) that IRQ
            break;
        case INTC_MASK_ADDR:
            reg_name = "INTC_MASK";
            intc_mask ^= value;   // Writing '1' to a bit reverses its current mask state
            break;
        default:
            Logger::error("Invalid INTC register write at address 0x" + format("{:08X}", address));
            Neo2::exit(1, Neo2::Subsystem::EE_INTC);
    }
    Logger::info("INTC register write to " + reg_name + " with value 0x" + format("{:08X}", value));
}

void EE_INTC::request_interrupt(uint32_t irq) {
    if (irq > 31) {
        Logger::error("Invalid IRQ number");
        Neo2::exit(1, Neo2::Subsystem::EE_INTC);
    }
    intc_stat |= (1 << irq);
}

void EE_INTC::acknowledge_interrupt(uint32_t irq) {
    if (irq > 31) {
        Logger::error("Invalid IRQ number");
        Neo2::exit(1, Neo2::Subsystem::EE_INTC);
    }
    intc_stat &= ~(1 << irq);
}

bool EE_INTC::is_interrupt_pending() const {
    // INT0 is asserted when (INTC_STAT & INTC_MASK) != 0
    return (intc_stat & intc_mask) != 0;
}
