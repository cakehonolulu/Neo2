#include "imgui.h"
#include <cpu/disassembler.hh>
#include <frontends/imgui/imgui_disassembler.hh>
#include <vector>
#include <iomanip>
#include <sstream>

ImGuiDisassembler::ImGuiDisassembler(ImGui_Neo2& neo2_instance, Disassembler& disassembler_instance)
    : neo2(neo2_instance), disassembler(disassembler_instance), start_pc(neo2_instance.ee.pc) {}

std::string ImGuiDisassembler::format_bytes(uint32_t opcode) {
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0');
    ss << std::setw(2) << ((opcode >> 24) & 0xFF) << " ";
    ss << std::setw(2) << ((opcode >> 16) & 0xFF) << " ";
    ss << std::setw(2) << ((opcode >> 8) & 0xFF) << " ";
    ss << std::setw(2) << (opcode & 0xFF);
    return ss.str();
}

void ImGuiDisassembler::render_disassembly() {
    const float min_disassembly_width = 400.0f;
    ImGui::SetNextWindowSizeConstraints(ImVec2(min_disassembly_width, 0), ImVec2(FLT_MAX, FLT_MAX));

    ImGui::Begin("EE Disassembler");

    bool pseudos = disassembler.pseudos;
    if (ImGui::Checkbox("Use Pseudocodes", &pseudos)) {
        disassembler.set_pseudos(pseudos);
    }

    ImGui::BeginChild("DisassemblyView", ImVec2(0, ImGui::GetContentRegionAvail().y), true, ImGuiWindowFlags_HorizontalScrollbar);

    // Dynamically determine instructions per page based on the current window height
    const float line_height = ImGui::GetTextLineHeightWithSpacing(); // Height per disassembled line
    const int instructions_per_page = static_cast<int>(ImGui::GetContentRegionAvail().y / line_height);

    // Limit scroll offset so that it doesn’t go out of bounds
    scroll_offset = std::max(0, scroll_offset);

    for (int i = 0; i < instructions_per_page; ++i) {
        uint32_t pc = start_pc + (scroll_offset + i) * 4;
        uint32_t opcode = neo2.bus.read32(pc);
        DisassemblyData data = disassembler.disassemble(pc, opcode);

        std::string symbol = disassembler.get_symbol_name(pc);
        if (!symbol.empty()) {
            ImGui::Text("0x%08X: <%s>", pc, symbol.c_str());
        }

        bool is_current_instruction = (pc == neo2.ee.pc);
        bool is_jump_delay_slot = (is_current_instruction && data.is_jump);

        ImVec4 text_color = is_current_instruction ? ImVec4(1.0f, 1.0f, 0.0f, 1.0f) :
                            (is_jump_delay_slot ? ImVec4(1.0f, 0.5f, 0.0f, 1.0f) : ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_Text, text_color);

        std::string mnemonic_line = data.mnemonic;
        bool first_operand = true;
        for (const auto& operand : data.operands) {
            mnemonic_line += first_operand ? " " : ", ";
            mnemonic_line += operand.text;
            first_operand = false;
        }

        ImGui::Text("0x%08X: %-20s", pc, mnemonic_line.c_str());
        ImGui::PopStyleColor();

        ImVec4 opcode_color = is_current_instruction ? ImVec4(1.0f, 1.0f, 0.0f, 1.0f) : ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_Text, opcode_color);

        float line_width = ImGui::GetContentRegionAvail().x;
        ImGui::SameLine(line_width - 90.0f);
        ImGui::Text("%s", format_bytes(opcode).c_str());
        ImGui::PopStyleColor();

        if (data.is_jump && pc == neo2.ee.pc) {
            uint32_t target_pc = data.jump_target;
            std::string target_symbol = disassembler.get_symbol_name(target_pc);
            ImGui::Text("    -> 0x%08X %s", target_pc, target_symbol.empty() ? "" : target_symbol.c_str());
        }
    }

    // Smooth scroll: Adjust scroll offset with mouse wheel input, making scrolling more fluid
    float scroll_delta = ImGui::GetIO().MouseWheel;
    if (scroll_delta > 0) {
        scroll_offset = std::max(0, scroll_offset - static_cast<int>(scroll_delta * instructions_per_page * 0.1f));
    } else if (scroll_delta < 0) {
        scroll_offset += static_cast<int>(-scroll_delta * instructions_per_page * 0.1f);
    }

    ImGui::EndChild();
    ImGui::End();
}