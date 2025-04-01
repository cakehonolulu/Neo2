#include <firewire/firewire.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif
#include <neo2.hh>

FireWire::FireWire()
{
    Logger::set_subsystem("FireWire");
}

uint32_t FireWire::read(uint32_t address)
{
    std::string reg_name;

    switch (address & 0xFF) {
        default:
            Logger::error("Invalid FireWire register read at address 0x" + format("{:08X}", address));
            break;
    }

    return Neo2::exit(1, Neo2::Subsystem::IOP);
}

void FireWire::write(uint32_t address, uint32_t value)
{
    std::string reg_name;

    switch (address & 0xFF) {
        case ILinkMMIO::PHY_Access: {
            ilink_regs.phy_reg.raw = value;
            if (ilink_regs.phy_reg.rd && !ilink_regs.phy_reg.wr)
            {
                // Register read
                Logger::error("Unimplemented FireWire PHY Access register read!");
                Neo2::exit(1, Neo2::Subsystem::Firewire);
            }
            else if (ilink_regs.phy_reg.wr && !ilink_regs.phy_reg.rd)
            {
                // Register write
                switch (ilink_regs.phy_reg.rx_addr)
                {
                    default:
                        Logger::error("Unimplemented FireWire PHY Access register write to reg: " +
                                      format("{:01X}", (std::uint8_t) ilink_regs.phy_reg.rx_addr));
                        Neo2::exit(1, Neo2::Subsystem::Firewire);
                        break;
                }
            }
            else
            {
                Logger::error("No write/read bit set on FireWire PHY Access value!");
                Neo2::exit(1, Neo2::Subsystem::Firewire);
            }
            break;
        }
        default:
            Logger::error("Invalid FireWire register write at address 0x" + format("{:08X}", address));
            break;
    }
}