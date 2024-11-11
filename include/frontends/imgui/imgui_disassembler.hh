#pragma once

#include <cpu/disassembler.hh>
#include <frontends/imgui/imgui_neo2.h>

class ImGuiDisassembler {
  public:
    ImGuiDisassembler(ImGui_Neo2& neo2_instance, Disassembler& disassembler_instance);
    void render_disassembly();
    void render_cpu_disassembly(const char* window_name, uint32_t start_pc, CPU* cpu, bool& pseudos, int& scroll_offset);

    bool use_pseudos_ee = true;
    bool use_pseudos_iop = true;
    int scroll_offset_ee = 0;
    int scroll_offset_iop = 0;
    bool error_state_active = false;

  private:
    ImGui_Neo2& neo2;
    Disassembler& disassembler;
    uint32_t start_pc;

    std::string format_bytes(uint32_t opcode);
};
