#pragma once

#include <cstdint>

enum class RunType
{
    Step,
    Run
};

constexpr double PS2_FRAME_RATE = 59.94; // NTSC frame rate
constexpr uint64_t PS2_CYCLES_PER_FRAME = 4915200; // Cycles per frame at 294.912 MHz
constexpr uint64_t EE_CLOCKRATE = 294912000; // 294.912 MHz
constexpr uint64_t GS_VBLANK_DELAY = 65622;
constexpr uint64_t VBLANK_START_CYCLES = 4489019;