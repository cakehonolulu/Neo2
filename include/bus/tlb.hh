#pragma once

#include <log/log.hh>
#include <cstdint>
#include <vector>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

struct TLBEntry {
    uint32_t page_mask;
    uint32_t entry_hi;
    uint32_t entry_lo0;
    uint32_t entry_lo1;
    bool global;

    bool operator!=(const TLBEntry& other) const {
        return entry_hi != other.entry_hi ||
               entry_lo0 != other.entry_lo0 ||
               entry_lo1 != other.entry_lo1 ||
               page_mask != other.page_mask ||
               global != other.global;
    }

    bool operator==(const TLBEntry& other) const {
        return !(*this != other);
    }
};

class TLB {
public:
    TLB(size_t num_entries) : entries(num_entries) {}

    void write_entry(uint32_t index, const TLBEntry& entry) {
        Logger::info("Writing TLB entry at index 0x" + format("{:08X}", index) +
                    ", EntryHi: 0x" + format("{:08X}", entry.entry_hi) +
                    ", EntryLo0: 0x" + format("{:08X}", entry.entry_lo0) +
                    ", EntryLo1: 0x" + format("{:08X}", entry.entry_lo1));
        if (index < entries.size()) {
            entries[index] = entry;
        }
    }

    const TLBEntry* find_entry(uint32_t vaddr) const {
        for (const auto& entry : entries) {
            // Mask the VPN2 field according to the PageMask
            uint32_t mask = entry.page_mask;  // Use page_mask directly here
            uint32_t vpn2 = (vaddr & 0xFFFFF000); // Extract the VPN2 from vaddr

            if ((vpn2 & mask) == (entry.entry_hi & mask)) {
                return &entry;
            }
        }
        return nullptr;
    }


    const std::vector<TLBEntry>& get_tlb_entries() const { return entries; }

private:
    std::vector<TLBEntry> entries;
};
