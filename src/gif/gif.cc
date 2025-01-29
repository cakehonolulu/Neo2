#include <gif/gif.hh>
#include <log/log.hh>
#include <gs/gs.hh>
#include <bus/bus.hh>
#include <neo2.hh>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

GIF::GIF(Bus& bus) : bus(bus), gif_ctrl(0), gif_mode(0), gif_stat(0), gif_cnt(0), gif_p3cnt(0), gif_p3tag(0), state(State::Idle), nloop(0), current_nloop(0), nregs(0) {
    Logger::set_subsystem("GIF");
    for (int i = 0; i < 4; ++i) {
        gif_tag[i] = 0;
        gif_fifo[i] = 0;
    }
    current_gif_tag.u128 = 0;
}

uint32_t GIF::read(uint32_t address) {
    std::string reg_name;
    switch (address) {
        case GIF_STAT:
            reg_name = "GIF_STAT";
            break;
        case GIF_TAG0:
            reg_name = "GIF_TAG0";
            break;
        case GIF_TAG1:
            reg_name = "GIF_TAG1";
            break;
        case GIF_TAG2:
            reg_name = "GIF_TAG2";
            break;
        case GIF_TAG3:
            reg_name = "GIF_TAG3";
            break;
        case GIF_CNT:
            reg_name = "GIF_CNT";
            break;
        case GIF_P3CNT:
            reg_name = "GIF_P3CNT";
            break;
        case GIF_P3TAG:
            reg_name = "GIF_P3TAG";
            break;
        case GIF_FIFO:
        case GIF_FIFO + 4:
        case GIF_FIFO + 8:
        case GIF_FIFO + 12:
            reg_name = format("GIF_FIFO+{}", (address - GIF_FIFO) / 4);
            break;
        default:
            Logger::error("Invalid GIF register read at address 0x" + format("{:08X}", address));
            return 0;
    }
    Logger::info("GIF register read from " + reg_name);
    return (reg_name.rfind("GIF_FIFO", 0) == 0) ? gif_fifo[(address - GIF_FIFO) / 4] : *(reinterpret_cast<uint32_t*>(reinterpret_cast<uint8_t*>(this) + (address - GIF_CTRL)));
}

void GIF::write(uint32_t address, uint32_t value) {
    std::string reg_name;
    switch (address) {
        case GIF_CTRL:
            reg_name = "GIF_CTRL";
            gif_ctrl = value;
            if (value & 0x1) {
                // Reset GIF
                gif_stat = 0;
                for (int i = 0; i < 4; ++i) {
                    gif_tag[i] = 0;
                }
                gif_cnt = 0;
                gif_p3cnt = 0;
                gif_p3tag = 0;
                state = State::Idle;
            }
            break;
        case GIF_MODE:
            reg_name = "GIF_MODE";
            gif_mode = value;
            break;
        case GIF_FIFO:
        case GIF_FIFO + 4:
        case GIF_FIFO + 8:
        case GIF_FIFO + 12:
            reg_name = format("GIF_FIFO+{}", (address - GIF_FIFO) / 4);
            gif_fifo[(address - GIF_FIFO) / 4] = value;
            break;
        default:
            Logger::error("Invalid GIF register write at address 0x" + format("{:08X}", address));
            return;
    }
    Logger::info("GIF register write to " + reg_name + " with value 0x" + format("{:08X}", value));
}

void GIF::write_fifo(uint128_t data, uint32_t &madr, uint32_t &qwc) {
    if (is_path3_masked()) {
        Logger::warn("PATH3 is masked; ignoring GIF FIFO write");
        return;
    }

    process_gif_data(data, madr, qwc);
}

bool GIF::is_path3_masked() const {
    return gif_mode & 0x1;
}

void GIF::process_gif_data(uint128_t data, uint32_t &madr, uint32_t &qwc) {
    switch (state) {
        case State::Idle:
        {
            Logger::debug("Raw GIFTag: 0x" + format("{:016X}", data.u64[1]) + format("{:016X}", data.u64[0]));
            current_gif_addr = madr;

            // Parse GIFTag
            uint64_t low = data.u64[0];

            nloop = low & 0x7FFF;          // Bits 0-14
            bool eop = (low >> 15) & 0x1;  // Bit 15
            bool enable_prim = (low >> 46) & 0x1; // Bit 46
            uint16_t prim = (low >> 47) & 0x7FF; // Bits 47-57
            uint8_t format = (low >> 58) & 0x3; // Bits 58-59
            nregs = (low >> 60) & 0xF; // Bits 60-63

            current_gif_tag.u128 = data.u128;

            Logger::info("Processing GIFtag: NLOOP=" + std::to_string(nloop) +
                         ", EOP=" + std::to_string(eop) +
                         ", NREGS=" + std::to_string(nregs) +
                         ", Format=" + std::to_string(format));

            // If NLOOP is 0, ignore the GIFtag
            if (nloop == 0) {
                Logger::warn("NLOOP is zero; skipping GIFtag");
                return;
            }

            // Handle PRIM if enabled
            if (enable_prim) {
                Logger::info("PRIM enabled; writing to GS PRIM register");
                bus.gs.write_internal_reg(0, prim);
            }

            switch (format) {
                case 0: // PACKED Format
                    state = State::ProcessingPacked;
                    current_nloop = nloop;
                    break;

                case 2: // IMAGE Format
                    state = State::ProcessingImage;
                    current_nloop = nloop;
                    break;

                default:
                    Logger::error("Unsupported GIF data format: " + std::to_string(format));
                    Neo2::exit(1, Neo2::Subsystem::GIF);
                    break;
            }
            break;
        }

        case State::ProcessingPacked:
            current_gif_addr += 16;
            process_packed_format();
            break;

        case State::ProcessingImage:
            process_image_format(data);
            break;

        default:
            Logger::error("Invalid GIF state");
            Neo2::exit(1, Neo2::Subsystem::GIF);
            break;
    }
}

void GIF::process_packed_format() {
    Logger::info("Processing PACKED format data: NLOOP=" + std::to_string(nloop) +
                 " (Current NLOOP=" + std::to_string(current_nloop) + ")" +
                 ", NREGS=" + std::to_string(nregs));

    for (uint32_t reg = nregs; reg > 0; --reg) {
        uint128_t data = bus.read128(current_gif_addr); // Simulate reading from FIFO
        uint32_t reg_index = (current_gif_tag.u64[1] >> ((nregs - reg) << 2)) & 0xF;
        bus.gs.write_packed_gif_data(reg_index, data);
    }
    current_nloop--;

    if (current_nloop == 0) {
        state = State::ProcessPackedEnd;
        Logger::info("End of packed reached");
        if (!((current_gif_tag.u64[0] >> 15) & 0x1))
        {
            Logger::info("GIF Packed transfer complete, still not EOP, continuing processing...");
        }
        else
        {
            gif_ctrl |= 0x1; // Update GIF_CTRL to indicate the transfer is complete
        }
        state = State::Idle;
    }
}

void GIF::process_image_format(uint128_t data) {
    Logger::info("Processing IMAGE format data: NLOOP=" + std::to_string(nloop));

    bus.gs.write_hwreg(data.u64[0]);
    bus.gs.write_hwreg(data.u64[1]);

    current_nloop--;

    if (current_nloop == 0) {
        state = State::ProcessPackedEnd;
        Logger::info("End of packed reached");
        if (!((current_gif_tag.u64[0] >> 15) & 0x1))
        {
            Logger::info("GIF Packed transfer complete, still not EOP, continuing processing...");
        }
        else
        {
            gif_ctrl |= 0x1; // Update GIF_CTRL to indicate the transfer is complete
        }
        state = State::Idle;
    }
}
