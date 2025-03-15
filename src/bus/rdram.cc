#include <bus/rdram.hh>
#include <log/log.hh>
#include <neo2.hh>

RDRAM::RDRAM()
{
    Logger::set_subsystem("RDRAM");
}

uint32_t RDRAM::read(uint32_t address)
{
    std::string reg_name;
    switch (address)
    {
        case MCH_RICM:
            reg_name = "MCH_RICM";
            break;
        case MCH_DRD:
            reg_name = "MCH_DRD";
            break;
        default:
            Logger::error("Invalid RDRAM register read at address 0x" + format("{:08X}", address));
            Neo2::exit(1, Neo2::Subsystem::RDRAM);
    }
    Logger::info("RDRAM register read from " + reg_name);
    return (address == MCH_RICM) ? mch_ricm_.value : mch_drd_;
}

void RDRAM::write(uint32_t address, uint32_t value)
{
    mch_ricm_.busy = 1;

    std::string reg_name;
    switch (address)
    {
        case MCH_RICM:
            reg_name = "MCH_RICM";
            mch_ricm_.value = value;
            cmd();
            break;
        case MCH_DRD:
            reg_name = "MCH_DRD";
            mch_drd_ = value;
            break;
        default:
            Logger::error("Invalid RDRAM register write at address 0x" + format("{:08X}", address));
            Neo2::exit(1, Neo2::Subsystem::RDRAM);
    }

    Logger::info("RDRAM register write to " + reg_name + " with value 0x" + format("{:08X}", value));
}

void RDRAM::cmd()
{
    uint32_t sop = mch_ricm_.sop;
    switch (sop)
    {
        case RDRAMCommand::SRD:
            Logger::info("RDRAM controller SDR command");
            srd();
            break;

        case RDRAMCommand::SWR:
            Logger::info("RDRAM controller SWR command");
            swr();
            break;

        case RDRAMCommand::SETR:
            Logger::info("RDRAM controller SETR command");
            setr();
            break;

        case 0b1110:
            Logger::info("RDRAM controller RSRV command");
            break;

        default:
            Logger::error("Unhandled RDRAM controller command: 0b" + format("{:04b}", sop));
            Neo2::exit(1, Neo2::Subsystem::RDRAM);
            break;
    }

    mch_ricm_.busy = 0;
}

void RDRAM::srd()
{
    /* Serial read of control register {SA11..SA0) of RDRAM (SDEV5..SDEV0). */
    switch ((mch_ricm_.value >> 16) & 0xFFF)
    {
        case RDRAMRegister::INIT:
        {
            bool found_ic = false;
            for (auto &ic : rdram_ic)
            {
                if (ic.init.sdevid == (((mch_ricm_.sdev_high << 5) | mch_ricm_.sdev_low)))
                {
                    mch_drd_ = ic.init.value;
                    found_ic = true;
                    Logger::info("RDRAM SRD register read from INIT");
                }
            }

            if (!found_ic)
            {
                Logger::error("RDRAM SRD register read from INIT failed");
                Neo2::exit(1, Neo2::Subsystem::RDRAM);
            }
            break;
        }

        default:
            Logger::error("Unhandled RDRAM SRD register: 0x" + format("{:04X}", (mch_ricm_.value >> 16) & 0xFFF));
            Neo2::exit(1, Neo2::Subsystem::RDRAM);
            break;
    }
}

void RDRAM::swr()
{
    /* Serial write of control register (SA11..SA0) of RDRAM (SDEV5..SDEV0). */
    switch ((mch_ricm_.value >> 16) & 0xFFF)
    {
        case RDRAMRegister::INIT:
            if (mch_ricm_.sbc)
            {
                for (auto &ic : rdram_ic)
                {
                    ic.init.value = mch_drd_ & 0x3FFF;
                }
            }
            else
            {
                rdram_ic[((mch_ricm_.sdev_high << 5) | mch_ricm_.sdev_low)].init.value = mch_drd_ & 0x1FFF;
            }
            Logger::info("RDRAM SWR register write to INIT");
            break;
            
        case RDRAMRegister::TCYCLE:
            if (mch_ricm_.sbc)
            {
                for (auto &ic : rdram_ic)
                {
                    ic.tcycle = mch_drd_ & 0x3F;
                }
            }
            else
            {
                rdram_ic[((mch_ricm_.sdev_high << 5) | mch_ricm_.sdev_low)].tcycle = mch_drd_ & 0x3F;
            }
            Logger::info("RDRAM SWR register write to TCYCLE");
            break;
            
        case RDRAMRegister::TEST77:
            if (mch_ricm_.sbc)
            {
                for (auto &ic : rdram_ic)
                {
                    ic.test77 = mch_drd_ & 0xFFFF;
                }
            }
            else
            {
                rdram_ic[((mch_ricm_.sdev_high << 5) | mch_ricm_.sdev_low)].test77 = mch_drd_ & 0xFFFF;
            }
            Logger::info("RDRAM SWR register write to TEST77");
            break;

        default:
            Logger::error("Unhandled RDRAM SWR register: 0x" + format("{:04X}", (mch_ricm_.value >> 16) & 0xFFF));
            Neo2::exit(1, Neo2::Subsystem::RDRAM);
            break;
    }
}

void RDRAM::setr()
{
    /*
        Set Reset bit; all control registers assume their reset values.
        a16 tSCYCLE delay until CLRR command.
    */

    if (mch_ricm_.sbc)
    {
        for (auto &ic : rdram_ic)
        {
            ic.init.value = 0;
            ic.test34 = 0;
            ic.cnfga.value = 0;
            ic.cnfgb.value = 0;
            ic.devid = 0;
            ic.refb = 0;
            ic.refr = 0;
            ic.cca.value = 0;
            ic.ccb.value = 0;
            ic.napx.value = 0;
            ic.pdnxa = 0;
            ic.pdnx = 0;
            ic.tparm.value = 0;
            ic.tfrm = 0;
            ic.tcdly1 = 0;
            ic.tcycle = 0;
            ic.skip.value = 0;
            ic.test77 = 0;
            ic.test78 = 0;
            ic.test79 = 0;
        }
    }
    else
    {
        auto &ic = rdram_ic[((mch_ricm_.sdev_high << 5) | mch_ricm_.sdev_low)];

        ic.init.value = 0;
        ic.test34 = 0;
        ic.cnfga.value = 0;
        ic.cnfgb.value = 0;
        ic.devid = 0;
        ic.refb = 0;
        ic.refr = 0;
        ic.cca.value = 0;
        ic.ccb.value = 0;
        ic.napx.value = 0;
        ic.pdnxa = 0;
        ic.pdnx = 0;
        ic.tparm.value = 0;
        ic.tfrm = 0;
        ic.tcdly1 = 0;
        ic.tcycle = 0;
        ic.skip.value = 0;
        ic.test77 = 0;
        ic.test78 = 0;
        ic.test79 = 0;
    } 
}
