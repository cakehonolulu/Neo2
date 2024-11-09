#pragma once

#include <ee/ee_disassembler.hh>
#include <frontends/imgui/imgui_neo2.h>

class ImGuiDisassembler {
public:
    ImGuiDisassembler(ImGui_Neo2& neo2_instance, Disassembler& disassembler_instance);
    void render_disassembly();

private:
    ImGui_Neo2& neo2;
    Disassembler& disassembler;
    uint32_t start_pc;
    int scroll_offset = 0;

    std::string format_bytes(uint32_t opcode);
};
