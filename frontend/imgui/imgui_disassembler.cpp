#include "imgui.h"
#include <cpu/disassembler.hh>
#include <frontends/imgui/imgui_disassembler.hh>
#include <vector>
#include <iomanip>
#include <sstream>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

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

void ImGuiDisassembler::render_cpu_disassembly(const char* window_name, uint32_t base_pc, CPU* cpu, bool& pseudos, int& scroll_offset) {
    ImGui::Begin(window_name);

    if (ImGui::Checkbox("Use Pseudocodes", &pseudos)) {
    }

    ImGui::Separator();

    // Step button
    if (ImGui::Button("Step")) {
        cpu->step();
    }

    if (Neo2::is_aborted()) {
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            (dynamic_cast<EE*>(cpu)) ? dynamic_cast<EE*>(cpu)->reset() : dynamic_cast<IOP*>(cpu)->reset();
            Neo2::reset_aborted();
        }
    }

    ImGui::BeginChild("DisassemblyView", ImVec2(0, ImGui::GetContentRegionAvail().y), true, ImGuiWindowFlags_HorizontalScrollbar);

    const float line_height = ImGui::GetTextLineHeightWithSpacing();
    const int instructions_per_page = static_cast<int>(ImGui::GetContentRegionAvail().y / line_height);

    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)) {
        float scroll_delta = ImGui::GetIO().MouseWheel;
        scroll_offset = scroll_delta < 0 ? scroll_offset + 1 : scroll_offset - 2;
    }

    if (Neo2::is_aborted())
    {
        aborted_still = true;
    }

    if (aborted_still)
    {
        base_pc = (dynamic_cast<EE*>(cpu) ? dynamic_cast<EE*>(cpu)->old_pc : dynamic_cast<IOP*>(cpu)->old_pc);
    }

    uint32_t current_pc = (base_pc + scroll_offset * 4);

    for (int i = 0; i < instructions_per_page; ++i) {
        uint32_t pc = current_pc + i * 4;
        uint32_t opcode = cpu->bus->read32(pc);
        DisassemblyData data = disassembler.disassemble(cpu, pc, opcode, pseudos);

        std::string symbol = disassembler.get_symbol_name(pc);
        if (!symbol.empty()) {
            ImGui::Text("0x%08X: <%s>", pc, symbol.c_str());
        }

        // Determine if this is the current instruction for highlighting
        uint32_t active_pc = (dynamic_cast<EE*>(cpu)) ? dynamic_cast<EE*>(cpu)->pc : dynamic_cast<IOP*>(cpu)->pc;

        if (aborted_still)
        {
            active_pc = (dynamic_cast<EE*>(cpu) ? dynamic_cast<EE*>(cpu)->old_pc : dynamic_cast<IOP*>(cpu)->old_pc);
        }

        bool is_current_instruction = (pc == active_pc);

        bool is_guilty_instruction = (aborted_still &&
                                      Neo2::get_guilty_subsystem() == ((dynamic_cast<EE*>(cpu)) ? Neo2::Subsystem::EE : Neo2::Subsystem::IOP) &&
                                      (dynamic_cast<EE*>(cpu) ? dynamic_cast<EE*>(cpu)->old_pc : dynamic_cast<IOP*>(cpu)->old_pc) == pc);

        ImVec4 text_color;
        if (is_guilty_instruction) {
            text_color = ImVec4(1.0f, 0.0f, 0.0f, 1.0f); // Red for the exact guilty instruction
        } else if (is_current_instruction) {
            text_color = ImVec4(1.0f, 1.0f, 0.0f, 1.0f); // Yellow for current instruction
        } else {
            text_color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f); // Default for others
        }

        ImGui::PushStyleColor(ImGuiCol_Text, text_color);

        // Render the disassembled line
        std::string mnemonic_line = data.mnemonic;
        bool first_operand = true;
        for (const auto& operand : data.operands) {
            mnemonic_line += first_operand ? " " : ", ";
            mnemonic_line += operand.text;
            first_operand = false;
        }

        ImGui::Text("0x%08X: %-20s", pc, mnemonic_line.c_str());
        ImGui::PopStyleColor();

        // Set the opcode color to gray by default
        ImVec4 opcode_color = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);  // Gray for all but the current instruction
        if (is_current_instruction || is_guilty_instruction) {
            opcode_color = text_color;
        }

        ImGui::PushStyleColor(ImGuiCol_Text, opcode_color);
        float line_width = ImGui::GetContentRegionAvail().x;
        ImGui::SameLine(line_width - 90.0f);
        ImGui::Text("%s", format_bytes(opcode).c_str());
        ImGui::PopStyleColor();

        if (data.is_jump && pc == active_pc) {
            uint32_t target_pc = data.jump_target;
            std::string target_symbol = disassembler.get_symbol_name(target_pc);
            ImGui::Text("    -> 0x%08X %s", target_pc, target_symbol.empty() ? "" : target_symbol.c_str());
        }
    }

    ImGui::EndChild();
    ImGui::End();
}
