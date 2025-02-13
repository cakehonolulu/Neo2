#pragma once

#include <cstdint>
#include <mutex>
#include <unordered_set>
#include <log/log.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

enum class CoreType {
    EE,   // Emotion Engine (EE) core
    IOP   // Input/Output Processor (IOP) core
};

class Breakpoint {
public:
    void notify_breakpoint(uint32_t address) {
        std::lock_guard<std::mutex> lock(breakpoint_mutex);
        Logger::info("Breakpoint hit at address: 0x" + format("{:08X}", address));
        breakpoint_hit = true;
        breakpoint_address = address;
    }

    void clear_breakpoint_notification() {
        std::lock_guard<std::mutex> lock(breakpoint_mutex);
        breakpoint_hit = false;
        breakpoint_address = 0;
    }

    bool is_breakpoint_hit() {
        std::lock_guard<std::mutex> lock(breakpoint_mutex);
        return breakpoint_hit;
    }

    uint32_t get_breakpoint_address() {
        std::lock_guard<std::mutex> lock(breakpoint_mutex);
        return breakpoint_address;
    }

    void add_breakpoint(uint32_t address, CoreType core) {
        std::lock_guard<std::mutex> lock(breakpoint_mutex);
        if (core == CoreType::EE) {
            ee_breakpoints.insert(address);
        } else if (core == CoreType::IOP) {
            iop_breakpoints.insert(address);
        }
    }

    void remove_breakpoint(uint32_t address, CoreType core) {
        std::lock_guard<std::mutex> lock(breakpoint_mutex);
        if (core == CoreType::EE) {
            ee_breakpoints.erase(address);
        } else if (core == CoreType::IOP) {
            iop_breakpoints.erase(address);
        }
    }

    bool has_breakpoint(uint32_t address, CoreType core) {
        std::lock_guard<std::mutex> lock(breakpoint_mutex);
        if (core == CoreType::EE) {
            return ee_breakpoints.count(address) > 0;
        } else if (core == CoreType::IOP) {
            return iop_breakpoints.count(address) > 0;
        }
        return false;
    }

    std::unordered_set<uint32_t> ee_breakpoints;
    std::unordered_set<uint32_t> iop_breakpoints;

private:
    std::mutex breakpoint_mutex;
    bool breakpoint_hit = false;
    uint32_t breakpoint_address = 0;
};
