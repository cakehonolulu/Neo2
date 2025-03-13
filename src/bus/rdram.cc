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
        case MCH_DRD:
            reg_name = "MCH_DRD";
            break;
        case MCH_RICM:
            reg_name = "MCH_RICM";
            break;
        default:
            Logger::error("Invalid RDRAM register read at address 0x" + format("{:08X}", address));
            Neo2::exit(1, Neo2::Subsystem::RDRAM);
    }
    Logger::info("RDRAM register read from " + reg_name);
    return (address == MCH_DRD) ? mch_drd_.value : mch_ricm_;
}

void RDRAM::write(uint32_t address, uint32_t value)
{
    mch_drd_.busy = 1;

    std::string reg_name;
    switch (address)
    {
    case MCH_DRD:
        reg_name = "MCH_DRD";
        mch_drd_.value = value;
        cmd();
        break;
    case MCH_RICM:
        reg_name = "MCH_RICM";
        mch_ricm_ = value;
        break;
    default:
        Logger::error("Invalid RDRAM register write at address 0x" + format("{:08X}", address));
        Neo2::exit(1, Neo2::Subsystem::RDRAM);
    }

    Logger::info("RDRAM register write to " + reg_name + " with value 0x" + format("{:08X}", value));
}

void RDRAM::cmd()
{
    uint32_t sop = mch_drd_.sop;
    switch (sop)
    {
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

    mch_drd_.busy = 0;
}

void RDRAM::swr()
{
    /* Serial write of control register (SA11..SA0) of RDRAM (SDEV5..SDEV0). */
}

void RDRAM::setr()
{
    /*
        Set Reset bit; all control registers assume their reset values.
        a16 tSCYCLE delay until CLRR command.
    */
    init.value = 0;
    test34 = 0;
    cnfga.value = 0;
    cnfgb.value = 0;
    devid = 0;
    refb = 0;
    refr = 0;
    cca.value = 0;
    ccb.value = 0;
    napx.value = 0;
    pdnxa = 0;
    pdnx = 0;
    tparm.value = 0;
    tfrm = 0;
    tcdly1 = 0;
    tcycle = 0;
    skip.value = 0;
    test77 = 0;
    test78 = 0;
    test79 = 0;
}
