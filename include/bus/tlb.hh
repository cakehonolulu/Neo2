#pragma once

#include <cstdint>
#include <vector>

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
