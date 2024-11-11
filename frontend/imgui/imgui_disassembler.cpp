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

    ImGui::Checkbox("Use Pseudocodes", &pseudos);

    ImGui::Separator();

    if (ImGui::Button("Step")) {
        cpu->step();
    }

    if (Neo2::is_aborted()) {
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            if (auto* ee = dynamic_cast<EE*>(cpu)) {
                ee->reset();
            } else if (auto* iop = dynamic_cast<IOP*>(cpu)) {
                iop->reset();
            }
            Neo2::reset_aborted();
            error_state_active = false;
        } else {
            error_state_active = true;
        }
    }

    ImGui::BeginChild("DisassemblyView", ImVec2(0, ImGui::GetContentRegionAvail().y), true, ImGuiWindowFlags_HorizontalScrollbar);
    const float line_height = ImGui::GetTextLineHeightWithSpacing();
    const int instructions_per_page = static_cast<int>(ImGui::GetContentRegionAvail().y / line_height);

    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)) {
        scroll_offset += static_cast<int>(ImGui::GetIO().MouseWheel * -1);
    }

    uint32_t display_pc;
    if (error_state_active) {
        if (auto* ee = dynamic_cast<EE*>(cpu)) {
            display_pc = ee->old_pc;
        } else if (auto* iop = dynamic_cast<IOP*>(cpu)) {
            display_pc = iop->old_pc;
        } else {
            display_pc = base_pc;
        }
    } else {
        display_pc = base_pc;
    }

    uint32_t current_pc = display_pc + scroll_offset * 4;

    // Render instructions
    for (int i = 0; i < instructions_per_page; ++i) {
        uint32_t pc = current_pc + i * 4;
        uint32_t opcode = cpu->bus->read32(pc);
        DisassemblyData data = disassembler.disassemble(cpu, pc, opcode, pseudos);

        // Get any symbols for this address
        std::string symbol = disassembler.get_symbol_name(pc);
        if (!symbol.empty()) {
            ImGui::Text("0x%08X: <%s>", pc, symbol.c_str());
        }

        // Determine colors for highlighting
        uint32_t active_pc = error_state_active
                                 ? (dynamic_cast<EE*>(cpu) ? dynamic_cast<EE*>(cpu)->old_pc : dynamic_cast<IOP*>(cpu)->old_pc)
                                 : (dynamic_cast<EE*>(cpu) ? dynamic_cast<EE*>(cpu)->pc : dynamic_cast<IOP*>(cpu)->pc);

        bool is_current_instruction = (pc == active_pc);
        bool is_guilty_instruction = (error_state_active &&
                                      Neo2::get_guilty_subsystem() == (dynamic_cast<EE*>(cpu) ? Neo2::Subsystem::EE : Neo2::Subsystem::IOP) &&
                                      active_pc == pc);

        ImVec4 text_color = is_guilty_instruction ? ImVec4(1.0f, 0.0f, 0.0f, 1.0f) :  // Red for guilty instruction
                                is_current_instruction ? ImVec4(1.0f, 1.0f, 0.0f, 1.0f) :  // Yellow for current instruction
                                ImVec4(1.0f, 1.0f, 1.0f, 1.0f);                            // Default color

        ImGui::PushStyleColor(ImGuiCol_Text, text_color);

        // Render the disassembled line
        std::string mnemonic_line = data.mnemonic;
        for (const auto& operand : data.operands) {
            mnemonic_line += (mnemonic_line.empty() ? "" : ", ") + operand.text;
        }

        ImGui::Text("0x%08X: %-20s", pc, mnemonic_line.c_str());
        ImGui::PopStyleColor();

        // Render opcode bytes
        ImVec4 opcode_color = is_guilty_instruction || is_current_instruction ? text_color : ImVec4(0.5f, 0.5f, 0.5f, 1.0f);  // Gray if unhighlighted
        ImGui::PushStyleColor(ImGuiCol_Text, opcode_color);
        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 90.0f);
        ImGui::Text("%s", format_bytes(opcode).c_str());
        ImGui::PopStyleColor();

        // Show jump target if applicable
        if (data.is_jump && pc == active_pc) {
            uint32_t target_pc = data.jump_target;
            std::string target_symbol = disassembler.get_symbol_name(target_pc);
            ImGui::Text("    -> 0x%08X %s", target_pc, target_symbol.empty() ? "" : target_symbol.c_str());
        }
    }

    ImGui::EndChild();
    ImGui::End();
}
