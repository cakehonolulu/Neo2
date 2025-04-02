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
        case i1934LLCCRegMap::NodeID: {
            return llc_core_regs.node_id_reg.value | 1;
            break;
        }

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
        case i1934LLCCRegMap::PHY_Access: {
            // RgDat, RgAdr, WrPhy & RdPhy are the only writable bitfields
            llc_core_regs.phy_access_reg.rgdat = ((value >> 16) & 0xF);
            llc_core_regs.phy_access_reg.rgadr = ((value >> 24) & 0xF);
            llc_core_regs.phy_access_reg.wrphy = ((value >> 30) & 1);
            llc_core_regs.phy_access_reg.rdphy = ((value >> 31) & 1);

            if (llc_core_regs.phy_access_reg.rdphy && !llc_core_regs.phy_access_reg.wrphy)
            {
                // Register read
                Logger::error("Unimplemented FireWire PHY Access register read!");
                Neo2::exit(1, Neo2::Subsystem::Firewire);
            }
            else if (llc_core_regs.phy_access_reg.wrphy && !llc_core_regs.phy_access_reg.rdphy)
            {
                // Register write
                switch (llc_core_regs.phy_access_reg.rgadr)
                {
                    case 0x01:
                        pl_core_regs.reg_01.value = llc_core_regs.phy_access_reg.rgdat;
                        break;

                    default:
                        Logger::error("Unimplemented FireWire PHY Access register write to reg: " +
                                      format("{:01X}", (std::uint8_t)llc_core_regs.phy_access_reg.rgadr));
                        Neo2::exit(1, Neo2::Subsystem::Firewire);
                        break;
                }

                // WrPhy bit is cleared once request to PHY has been sent
                llc_core_regs.phy_access_reg.wrphy = 0;
            }
            else
            {
                Logger::error("No write/read bit set on FireWire PHY Access value!");
                Neo2::exit(1, Neo2::Subsystem::Firewire);
            }
            break;
        }

        case i1934LLCCRegMap::Interrupt0:
            llc_core_regs.interrupt0.value = value;
            break;

        case i1934LLCCRegMap::Interrupt1:
            llc_core_regs.interrupt1.value = value;
            break;

        case i1934LLCCRegMap::Interrupt2:
            llc_core_regs.interrupt2.value = value;
            break;

        case i1934LLCCRegMap::UBUF_Receive_Clear:
            llc_core_regs.ubuf_recv_clear = value;
            break;

        default:
            Logger::error("Invalid FireWire register write at address 0x" + format("{:08X}", address));
            Neo2::exit(1, Neo2::Subsystem::Firewire);
            break;
    }
}