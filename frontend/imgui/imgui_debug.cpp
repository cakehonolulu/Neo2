#include "imgui.h"
#include <cpu/disassembler.hh>
#include <frontends/imgui/imgui_debug.hh>
#include <iomanip>
#include <sstream>
#include <unordered_map>

#if __has_include(<format>)
#include <format>
using std::format;
#else
#include <fmt/format.h>
using fmt::format;
#endif

ImGuiDebug::ImGuiDebug(ImGui_Neo2& neo2_instance, Disassembler& disassembler_instance)
    : neo2(neo2_instance), disassembler(disassembler_instance), start_pc(neo2_instance.ee.pc) {}

std::string ImGuiDebug::format_bytes(uint32_t opcode) {
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0');
    ss << std::setw(2) << ((opcode >> 24) & 0xFF) << " ";
    ss << std::setw(2) << ((opcode >> 16) & 0xFF) << " ";
    ss << std::setw(2) << ((opcode >> 8) & 0xFF) << " ";
    ss << std::setw(2) << (opcode & 0xFF);
    return ss.str();
}

// Render all debug windows
void ImGuiDebug::render_debug_windows() {
    if (ImGui::BeginTabBar("DebugTabs")) {
        if (ImGui::BeginTabItem("EE Debug")) {
            render_cpu_disassembly("EE Disassembler", neo2.ee.pc, &neo2.ee, use_pseudos_ee, scroll_offset_ee);
            render_cpu_registers("EE Registers", &neo2.ee);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("IOP Debug")) {
            render_cpu_disassembly("IOP Disassembler", neo2.iop.pc, &neo2.iop, use_pseudos_iop, scroll_offset_iop);
            render_cpu_registers("IOP Registers", &neo2.iop);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

// Render disassembly for a specific CPU
void ImGuiDebug::render_cpu_disassembly(const char* window_name, uint32_t base_pc, CPU* cpu, bool& pseudos, int& scroll_offset) {
    ImGui::Checkbox("Use Pseudocodes", &pseudos);

    ImGui::Separator();

    ImGui::BeginChild("DisassemblyView", ImVec2(0, ImGui::GetContentRegionAvail().y), true, ImGuiWindowFlags_HorizontalScrollbar);
    const float line_height = ImGui::GetTextLineHeightWithSpacing();
    const int instructions_per_page = static_cast<int>(ImGui::GetContentRegionAvail().y / line_height);

    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)) {
        scroll_offset += static_cast<int>(ImGui::GetIO().MouseWheel * -1);
    }

    uint32_t display_pc = base_pc;
    uint32_t current_pc = display_pc + scroll_offset * 4;

    for (int i = 0; i < instructions_per_page; ++i) {
        uint32_t pc = current_pc + i * 4;
        uint32_t opcode = cpu->bus->read32(pc);
        DisassemblyData data = disassembler.disassemble(cpu, pc, opcode, pseudos);

        std::string symbol = disassembler.get_symbol_name(pc);
        if (!symbol.empty()) {
            ImGui::Text("0x%08X: <%s>", pc, symbol.c_str());
        }

        uint32_t active_pc = (dynamic_cast<EE*>(cpu) ? dynamic_cast<EE*>(cpu)->pc : dynamic_cast<IOP*>(cpu)->pc);

        bool is_current_instruction = (pc == active_pc);
        bool is_guilty_instruction = (Neo2::is_aborted() &&
                                      Neo2::get_guilty_subsystem() == (dynamic_cast<EE*>(cpu) ? Neo2::Subsystem::EE : Neo2::Subsystem::IOP) &&
                                      active_pc == pc);

        ImVec4 text_color = is_guilty_instruction ? ImVec4(1.0f, 0.0f, 0.0f, 1.0f) :
                            is_current_instruction ? ImVec4(1.0f, 1.0f, 0.0f, 1.0f) :
                                                   ImVec4(1.0f, 1.0f, 1.0f, 1.0f);

        ImGui::PushStyleColor(ImGuiCol_Text, text_color);

        std::string mnemonic_line = data.mnemonic;
        for (const auto& operand : data.operands) {
            mnemonic_line += (mnemonic_line.empty() ? "" : ", ") + operand.text;
        }

        ImGui::Text("0x%08X: %-20s", pc, mnemonic_line.c_str());
        ImGui::PopStyleColor();

        ImVec4 opcode_color = is_guilty_instruction || is_current_instruction ? text_color : ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_Text, opcode_color);
        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 90.0f);
        ImGui::Text("%s", format_bytes(opcode).c_str());
        ImGui::PopStyleColor();

        if (data.is_jump && pc == active_pc) {
            uint32_t target_pc = data.jump_target;
            std::string target_symbol = disassembler.get_symbol_name(target_pc);
            ImGui::Text("    -> 0x%08X %s", target_pc, target_symbol.empty() ? "" : target_symbol.c_str());
        }
    }

    ImGui::EndChild();
}

const std::string mips_register_names[32] = {
    "zr", "at", "v0", "v1", "a0", "a1", "a2", "a3",
    "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
    "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
    "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"
};

const std::string cop0_register_names[32] = {
    "Index", "Random", "EntryLo0", "EntryLo1", "Context", "PageMask", "Wired", "Reserved",
    "BadVAddr", "Count", "EntryHi", "Compare", "Status", "Cause", "EPC", "PRId",
    "Config", "LLAddr", "WatchLo", "WatchHi", "XContext", "Reserved", "Reserved", "Reserved",
    "Reserved", "Reserved", "ECC", "CacheErr", "TagLo", "TagHi", "ErrorEPC", "Reserved"
};

std::unordered_map<int, uint128_t> previous_ee_registers;
std::unordered_map<int, std::uint32_t> previous_iop_registers;
std::unordered_map<int, float> register_change_timers;

void ImGuiDebug::render_cpu_registers(const char* window_name, CPU* cpu) {
    ImGui::Text("Registers");

    // General-purpose registers table
    if (ImGui::BeginTable("gpr_table", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        if (auto* ee = dynamic_cast<EE*>(cpu)) {
            for (int i = 0; i < 32; ++i) {
                ImGui::TableNextColumn();
                uint128_t current_value = ee->registers[i];
                if (previous_ee_registers[i].u128 != current_value.u128) {
                    previous_ee_registers[i] = current_value;
                    register_change_timers[i] = 1.0f; // Set timer to 1 second
                }
                ImVec4 color = ImVec4(1.0f, 1.0f - register_change_timers[i], 1.0f - register_change_timers[i], 1.0f);
                ImGui::PushStyleColor(ImGuiCol_Text, color);
                ImGui::Text("%-3s: 0x%016llX%016llX", mips_register_names[i].c_str(), current_value.u64[1], current_value.u64[0]);
                ImGui::PopStyleColor();
                if (register_change_timers[i] > 0.0f) {
                    register_change_timers[i] -= ImGui::GetIO().DeltaTime;
                }
            }
        } else if (auto* iop = dynamic_cast<IOP*>(cpu)) {
            for (int i = 0; i < 32; ++i) {
                ImGui::TableNextColumn();
                std::uint32_t current_value = iop->registers[i];
                if (previous_iop_registers[i] != current_value) {
                    previous_iop_registers[i] = current_value;
                    register_change_timers[i] = 1.0f; // Set timer to 1 second
                }
                ImVec4 color = ImVec4(1.0f, 1.0f - register_change_timers[i], 1.0f - register_change_timers[i], 1.0f);
                ImGui::PushStyleColor(ImGuiCol_Text, color);
                ImGui::Text("%-3s: 0x%08X", mips_register_names[i].c_str(), current_value);
                ImGui::PopStyleColor();
                if (register_change_timers[i] > 0.0f) {
                    register_change_timers[i] -= ImGui::GetIO().DeltaTime;
                }
            }
        }
        ImGui::EndTable();
    }

    ImGui::Separator();
    ImGui::Text("COP0 Registers");

    // COP0 registers table
    if (ImGui::BeginTable("cop0_table", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        if (auto* ee = dynamic_cast<EE*>(cpu)) {
            for (int i = 0; i < 32; ++i) {
                ImGui::TableNextColumn();
                ImGui::Text("%-8s: 0x%08X", cop0_register_names[i].c_str(), ee->cop0_registers[i]);
            }
        } else if (auto* iop = dynamic_cast<IOP*>(cpu)) {
            for (int i = 0; i < 32; ++i) {
                ImGui::TableNextColumn();
                ImGui::Text("%-8s: 0x%08X", cop0_register_names[i].c_str(), iop->cop0_registers[i]);
            }
        }
        ImGui::EndTable();
    }
}
