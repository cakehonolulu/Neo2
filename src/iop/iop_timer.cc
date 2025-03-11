#include <iop/iop_timer.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

IOP_Timer::IOP_Timer() {
    Logger::set_subsystem("IOP_Timer");
    for (int i = 0; i < 6; ++i) {
        timers[i].count = 0;
        timers[i].mode = 0;
        timers[i].target = 0;
    }
}

uint32_t IOP_Timer::read(uint32_t address) {
    std::string reg_name;
    uint32_t timer_index = (address >> 4) & 0x7; // Determine which timer

    switch (address & 0xFFF) {
        case 0x000:
            reg_name = format("TIMER_COUNT[{}]", timer_index);
            Logger::info("IOP Timer register read from " + reg_name);
            return timers[timer_index].count;

        case 0x004:
            reg_name = format("TIMER_MODE[{}]", timer_index);
            Logger::info("IOP Timer register read from " + reg_name);
            return timers[timer_index].mode;

        case 0x008:
            reg_name = format("TIMER_TARGET[{}]", timer_index);
            Logger::info("IOP Timer register read from " + reg_name);
            return timers[timer_index].target;

        default:
            Logger::error("Invalid IOP Timer register read at address 0x" + format("{:08X}", address));
            return 0;
    }
}

void IOP_Timer::write(uint32_t address, uint32_t value) {
    std::string reg_name;
    uint32_t timer_index = (address >> 4) & 0x7; // Determine which timer

    switch (address & 0xFFF) {
        case 0x000:
            reg_name = format("TIMER_COUNT[{}]", timer_index);
            Logger::info("IOP Timer register write to " + reg_name + " with value 0x" + format("{:08X}", value));
            timers[timer_index].count = value;
            break;

        case 0x004:
            reg_name = format("TIMER_MODE[{}]", timer_index);
            Logger::info("IOP Timer register write to " + reg_name + " with value 0x" + format("{:08X}", value));
            timers[timer_index].mode = value;
            break;

        case 0x008:
            reg_name = format("TIMER_TARGET[{}]", timer_index);
            Logger::info("IOP Timer register write to " + reg_name + " with value 0x" + format("{:08X}", value));
            timers[timer_index].target = value;
            break;

        default:
            Logger::error("Invalid IOP Timer register write at address 0x" + format("{:08X}", address));
            break;
    }
}

