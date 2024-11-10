#include <bus/bus.hh>
#include <bus/bus_fmem.hh>
#include <log/log.hh>
#include <cstring>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

void Bus::fmem_init()
{
    // PS2's Address Space is 4GB, divide it in 4KB pages
    address_space_r = new uintptr_t[0x100000];
    address_space_w = new uintptr_t[0x100000];

    const uint32_t PAGE_SIZE = 4 * 1024; // Page size = 4KB

    memset(address_space_r, 0, sizeof(uintptr_t) * 0x100000);
    memset(address_space_w, 0, sizeof(uintptr_t) * 0x100000);

    // 4MB's of BIOS memory fit in 1024, 4KB pages
    for (auto pageIndex = 0; pageIndex < 1024; pageIndex++)
    {
        const auto pointer = (uintptr_t)&bios[(pageIndex * PAGE_SIZE)]; // pointer to page #pageIndex of the BIOS
        address_space_r[pageIndex + 0x1FC00] = pointer;                 // map this page to KUSEG BIOS
        address_space_r[pageIndex + 0x9FC00] = pointer;                 // Same for KSEG0
        address_space_r[pageIndex + 0xBFC00] = pointer;                 // Same for KSEG1
    }
}

std::uint32_t Bus::fmem_read32(std::uint32_t address)
{
    const auto page = address >> 12;            // Divide the address by 4KB to get the page number.
    const auto offset = address & 0xFFF;        // The offset inside the 4KB page
    const auto pointer = address_space_r[page]; // Get the pointer to this page

    if (pointer != 0) // Check if the pointer is not nullptr. If it is not, then this is a fast page
	{
		return *(uint32_t *)(pointer + offset);
	}
    else
    {
        // Handle other cases or throw an exception if needed
        std::string msg;
        msg = "32-bit read from unknown address: 0x" + format("{:08X}", address);
        Logger::error(msg.c_str());
        return 0;
    }
}