#include <ee/timer/timer.hh>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

EE_Timer::EE_Timer() {
    Logger::set_subsystem("EE_Timer");
    for (int i = 0; i < 4; ++i) {
        timers[i].count = 0;
        timers[i].comp = 0;
        timers[i].mode = 0;
        if (i < 2) { // Only T0 and T1 have the hold register
            timers[i].hold = 0;
        }
    }
}

static int timer0 = 0;

uint32_t EE_Timer::read(uint32_t address) {
    std::string reg_name;
    uint32_t timer_index = (address >> 11) & 0x3; // Determine which timer

    switch (address & 0xFFF) {
        case 0x000:
            reg_name = format("TN_COUNT[{}]", timer_index);
            Logger::info("Timer register read from " + reg_name);
            if (timer_index == 0)
            {
                timer0 += 1;
                return timer0 >> 11;
            }
            return timers[timer_index].count;

        case 0x010:
            reg_name = format("TN_MODE[{}]", timer_index);
            Logger::info("Timer register read from " + reg_name);
            return timers[timer_index].mode;

        case 0x020:
            reg_name = format("TN_COMP[{}]", timer_index);
            Logger::info("Timer register read from " + reg_name);
            return timers[timer_index].comp;

        case 0x030:
            if (timer_index < 2) { // Only T0 and T1 have the hold register
                reg_name = format("TN_HOLD[{}]", timer_index);
                Logger::info("Timer register read from " + reg_name);
                return timers[timer_index].hold;
            } else {
                Logger::error("Invalid Timer register read at address 0x" + format("{:08X}", address));
                return 0;
            }

        default:
            Logger::error("Invalid Timer register read at address 0x" + format("{:08X}", address));
            return 0;
    }
}

void EE_Timer::write(uint32_t address, uint32_t value) {
    std::string reg_name;
    uint32_t timer_index = (address >> 11) & 0x3; // Determine which timer

    switch (address & 0xFFF) {
        case 0x000:
            reg_name = format("TN_COUNT[{}]", timer_index);
            Logger::info("Timer register write to " + reg_name + " with value 0x" + format("{:08X}", value));
            timers[timer_index].count = value & 0xFFFF; // Mask to 16 bits
            break;

        case 0x010:
            reg_name = format("TN_MODE[{}]", timer_index);
            Logger::info("Timer register write to " + reg_name + " with value 0x" + format("{:08X}", value));
            timers[timer_index].mode = value;
            break;

        case 0x020:
            reg_name = format("TN_COMP[{}]", timer_index);
            Logger::info("Timer register write to " + reg_name + " with value 0x" + format("{:08X}", value));
            timers[timer_index].comp = value & 0xFFFF; // Mask to 16 bits
            break;

        case 0x030:
            if (timer_index < 2) { // Only T0 and T1 have the hold register
                reg_name = format("TN_HOLD[{}]", timer_index);
                Logger::info("Timer register write to " + reg_name + " with value 0x" + format("{:08X}", value));
                timers[timer_index].hold = value & 0xFFFF; // Mask to 16 bits
            } else {
                Logger::error("Invalid Timer register write at address 0x" + format("{:08X}", address));
            }
            break;

        default:
            Logger::error("Invalid Timer register write at address 0x" + format("{:08X}", address));
            break;
    }
}
