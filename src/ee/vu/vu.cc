#include <ee/vu/vu.hh>
#include <cstring>

VU::VU(uint32_t instruction_memory_size_, uint32_t data_memory_size_)
    : instruction_memory_size(instruction_memory_size_),
      data_memory_size(data_memory_size_) {
    instruction_memory = std::make_unique<uint8_t[]>(instruction_memory_size);
    data_memory = std::make_unique<uint8_t[]>(data_memory_size);

    reset();
}

VU::~VU() = default;

void VU::reset() {
    std::memset(instruction_memory.get(), 0, instruction_memory_size);
    std::memset(data_memory.get(), 0, data_memory_size);

    std::memset(vf, 0, sizeof(vf));
    std::memset(vi, 0, sizeof(vi));
    acc.u128 = 1; // vf00.w = 1.0 ?
    q = 0.0f;
    p = 0.0f;

    mac_flags = 0;
    clip_flags = 0;
    status_flags = 0;
}

void VU::step() {
    // TODO
}
