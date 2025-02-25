#pragma once

#include <cstdint>
#include <log/log.hh>
#include <vector>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

struct TLBEntry
{
    // Even page fields (from EntryLo0)
    bool v0 : 1;        // Valid, bit 30 of EntryLo0
    bool d0 : 1;        // Dirty, bit 29
    uint8_t c0 : 3;     // Cache mode, bits 28-26
    uint32_t pfn0 : 20; // Page Frame Number, bits 25-6

    // Odd page fields (from EntryLo1)
    bool v1 : 1;        // Valid, bit 30 of EntryLo1
    bool d1 : 1;        // Dirty, bit 29
    uint8_t c1 : 3;     // Cache mode, bits 28-26
    uint32_t pfn1 : 20; // Page Frame Number, bits 25-6

    // Shared fields (derived from EntryHi and PageMask)
    bool scratchpad : 1;     // Taken from bit 31 of EntryHi
    uint8_t asid : 8;        // ASID, lower 8 bits of EntryHi
    bool global : 1;         // Global flag: G bit is the AND of the G bits in EntryLo0 and EntryLo1 (bit 31 each)
    uint32_t vpn2 : 19;      // VPN2: Bits 31-13 of EntryHi masked by ~ (PageMask >> 13)
    uint32_t page_mask : 12; // Lower 12 bits of PageMask

    /* COP0 register values, stored for reference */
    uint32_t entry_hi = 0;
    uint32_t entry_lo0 = 0;
    uint32_t entry_lo1 = 0;

    bool matches(uint32_t vaddr) const
    {
        uint32_t effective_mask = ~(page_mask >> 13);
        uint32_t v_vpn2 = (vaddr >> 13) & effective_mask;
        bool vpn_matches = (v_vpn2 == vpn2);
        bool asid_matches = global || ((vaddr >> 13) & 0xFF) == asid;
        return vpn_matches && asid_matches;
    }

    bool operator!=(const TLBEntry &other) const
    {
        return v0 != other.v0 || d0 != other.d0 || c0 != other.c0 || pfn0 != other.pfn0 || v1 != other.v1 ||
               d1 != other.d1 || c1 != other.c1 || pfn1 != other.pfn1 || scratchpad != other.scratchpad ||
               asid != other.asid || global != other.global || vpn2 != other.vpn2 || page_mask != other.page_mask;
    }

    bool operator==(const TLBEntry &other) const
    {
        return !(*this != other);
    }
};

class TLB
{
  public:
    TLB(size_t num_entries) : entries(num_entries)
    {
    }

    // Write a TLB entry to the specified index.
    void write_entry(uint32_t index, uint32_t PageMask, uint32_t EntryHi, uint32_t EntryLo0, uint32_t EntryLo1)
    {
        if (index < entries.size())
        {
            TLBEntry new_entry;
            new_entry.entry_hi = EntryHi;
            new_entry.entry_lo0 = EntryLo0;
            new_entry.entry_lo1 = EntryLo1;
            new_entry.page_mask = PageMask & 0xFFF; // lower 12 bits

            // Compute VPN2: Take bits 31..13 of EntryHi, then mask off the lower bits determined by PageMask.
            new_entry.vpn2 = (EntryHi >> 13) & ~(PageMask >> 13);

            // ASID: lower 8 bits of EntryHi.
            new_entry.asid = EntryHi & 0xFF;

            // Extract even page fields from EntryLo0:
            new_entry.pfn0 = (EntryLo0 >> 6) & 0xFFFFF; // Bits 6-25.
            new_entry.c0 = (EntryLo0 >> 26) & 0x7;      // Bits 26-28.
            new_entry.d0 = (EntryLo0 >> 29) & 0x1;      // Bit 29.
            new_entry.v0 = (EntryLo0 >> 30) & 0x1;      // Bit 30.
            // Extract odd page fields from EntryLo1:
            new_entry.pfn1 = (EntryLo1 >> 6) & 0xFFFFF; // Bits 6-25.
            new_entry.c1 = (EntryLo1 >> 26) & 0x7;      // Bits 26-28.
            new_entry.d1 = (EntryLo1 >> 29) & 0x1;      // Bit 29.
            new_entry.v1 = (EntryLo1 >> 30) & 0x1;      // Bit 30.

            // Global flag: AND the G bits from EntryLo0 and EntryLo1 (assumed to be bit 31 of each).
            bool g0 = ((EntryLo0 >> 31) & 0x1) != 0;
            bool g1 = ((EntryLo1 >> 31) & 0x1) != 0;
            new_entry.global = g0 && g1;

            // Scratchpad flag: taken from bit 31 of EntryHi.
            new_entry.scratchpad = (EntryHi >> 31) & 0x1;

            entries[index] = new_entry;
        }
    }

    void write_entry_(uint32_t index, const TLBEntry &entry)
    {
        if (index < entries.size())
        {
            entries[index] = entry;
        }
    }

    const TLBEntry *find_entry(uint32_t vaddr) const
    {
        for (const auto &entry : entries)
        {
            if (entry.matches(vaddr))
            {
                return &entry;
            }
        }
        return nullptr;
    }

    const std::vector<TLBEntry> &get_tlb_entries() const
    {
        return entries;
    }

  private:
    std::vector<TLBEntry> entries;
};
