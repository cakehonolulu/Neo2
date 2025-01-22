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

// Updated TLBEntry with separate variables for all fields
struct TLBEntry {
    // Even page fields
    bool v0 : 1;              // Bit 1: Even page valid
    bool d0 : 1;              // Bit 2: Even page dirty
    uint8_t c0 : 3;           // Bits 3-5: Even page cache mode
    uint32_t pfn0 : 20;       // Bits 6-25: Even page frame number

    // Odd page fields
    bool v1 : 1;              // Bit 33: Odd page valid
    bool d1 : 1;              // Bit 34: Odd page dirty
    uint8_t c1 : 3;           // Bit 35: Odd page cache mode
    uint32_t pfn1 : 20;       // Bits 38-57: Odd page frame number

    // Shared fields
    bool scratchpad : 1;      // Bit 63: Scratchpad flag
    uint8_t asid : 8;         // Bits 64-71: Address Space ID
    bool global : 1;          // Bit 76: Global flag (ignore ASID if set)
    uint32_t vpn2 : 19;       // Bits 77-95: Virtual page number / 2
    uint32_t page_mask : 12;  // Bits 109-120: Page size mask

    /* COP0 Representation */

    uint32_t entry_hi = 0;
    uint32_t entry_lo0 = 0;
    uint32_t entry_lo1 = 0;

    bool matches(uint32_t vaddr) const {
        uint32_t page_size = 1 << (page_mask);  // This gets the page size from the page_mask
        uint32_t vpn2_vaddr = (vaddr & ~(page_size - 1)) >> (13 + page_mask);  // Adjust by page size

        // Check if the virtual page number matches the entry's VPN2 and ASID
        bool vpn_matches = vpn2_vaddr == vpn2;
        bool asid_matches = global || ((vaddr >> 13) & 0xFF) == asid;

        return vpn_matches && asid_matches;
    }

    bool operator!=(const TLBEntry& other) const {
        return v0 != other.v0 || d0 != other.d0 || c0 != other.c0 || pfn0 != other.pfn0 ||
               v1 != other.v1 || d1 != other.d1 || c1 != other.c1 || pfn1 != other.pfn1 ||
               scratchpad != other.scratchpad || asid != other.asid || global != other.global ||
               vpn2 != other.vpn2 || page_mask != other.page_mask;
    }

    bool operator==(const TLBEntry& other) const {
        return !(*this != other);
    }
};

// TLB class for managing a set of TLB entries
class TLB {
public:
    TLB(size_t num_entries) : entries(num_entries) {}

    // Write a TLB entry to the specified index
    void write_entry(uint32_t index, uint32_t PageMask, uint32_t EntryHi, uint32_t EntryLo0, uint32_t EntryLo1) {
        if (index < entries.size()) {
            /*Logger::info("Writing TLB entry at index 0x" + format("{:08X}", index) +
                    ", EntryHi: 0x" + format("{:08X}", EntryHi) +
                    ", EntryLo0: 0x" + format("{:08X}", EntryLo0) +
                    ", EntryLo1: 0x" + format("{:08X}", EntryLo1) +
                    ", VPN2: 0x" + format("{:08X}", (EntryHi >> 13)));*/
            TLBEntry new_entry;
            new_entry.entry_hi = EntryHi;
            new_entry.entry_lo0 = EntryLo0;
            new_entry.entry_lo1 = EntryLo1;
            new_entry.page_mask = PageMask;
            new_entry.vpn2 = (EntryHi >> 13) & ~(PageMask >> 13);
            new_entry.asid = EntryHi & 0xFF;
            new_entry.v0 = EntryLo0 & 0x1;
            new_entry.d0 = (EntryLo0 >> 1) & 0x1;
            new_entry.c0 = (EntryLo0 >> 3) & 0x7;
            new_entry.pfn0 = (EntryLo0 >> 6) & 0xFFFFF;
            new_entry.v1 = EntryLo1 & 0x1;
            new_entry.d1 = (EntryLo1 >> 1) & 0x1;
            new_entry.c1 = (EntryLo1 >> 3) & 0x7;
            new_entry.pfn1 = (EntryLo1 >> 6) & 0xFFFFF;
            new_entry.global = (EntryLo0 & 0x100) && (EntryLo1 & 0x100);
            new_entry.scratchpad = (EntryHi >> 31) & 0x1;

            entries[index] = new_entry;
        }
    }

    // Write a TLB entry to the specified index
    void write_entry_(uint32_t index, const TLBEntry& entry) {
        if (index < entries.size()) {
            entries[index] = entry;
        }
    }

    // Find the TLB entry matching the given virtual address
    const TLBEntry* find_entry(uint32_t vaddr) const {
        for (const auto& entry : entries) {
            if (entry.matches(vaddr)) {
                return &entry;
            }
        }
        return nullptr;
    }

    // Return all TLB entries (for debugging or visualization)
    const std::vector<TLBEntry>& get_tlb_entries() const { return entries; }

private:
    std::vector<TLBEntry> entries;
};
