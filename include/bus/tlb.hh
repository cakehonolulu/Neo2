#pragma once

#include <cstdint>
#include <vector>

struct TLBEntry {
    uint32_t page_mask;
    uint32_t entry_hi;
    uint32_t entry_lo0;
    uint32_t entry_lo1;
    bool global;
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
        uint32_t mask = ~(entry.page_mask << 13);
        if ((vaddr & mask) == (entry.entry_hi & mask)) {
            return &entry;
        }
    }
    return nullptr;
}


private:
    std::vector<TLBEntry> entries;
};
