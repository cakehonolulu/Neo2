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
        bool first_operand = true;
        for (const auto& operand : data.operands) {
            if (!first_operand) {
                mnemonic_line += ", ";
            }
            else
            {
                mnemonic_line += " ";
            }
            mnemonic_line += operand.text;
            first_operand = false;
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
std::unordered_map<int, float> ee_register_change_timers;
std::unordered_map<int, float> iop_register_change_timers;

void ImGuiDebug::render_cpu_registers(const char* window_name, CPU* cpu) {
    ImGui::Text("Registers");

    ImGui::Separator();
    
    if (auto* ee = dynamic_cast<EE*>(cpu)) {
        // Text with current PC and next PC in parentheses casted from ee
        ImGui::Text("PC: 0x%08X (%08X)", ee->pc, ee->next_pc);
        
        // Display LO and HI registers for EE
        ImGui::Text("LO: 0x%08X", ee->lo);
        ImGui::Text("HI: 0x%08X", ee->hi);
        
    } else if (auto* iop = dynamic_cast<IOP*>(cpu)) {
        // Text with current PC and next PC in parentheses casted from iop
        ImGui::Text("PC: 0x%08X (%08X)", iop->pc, iop->next_pc);
    }

    ImGui::Separator();
    
    // General-purpose registers table
    if (ImGui::BeginTable("gpr_table", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        if (auto* ee = dynamic_cast<EE*>(cpu)) {
            for (int i = 0; i < 32; ++i) {
                ImGui::TableNextColumn();
                uint128_t current_value = ee->registers[i];
                if (previous_ee_registers[i].u128 != current_value.u128) {
                    previous_ee_registers[i] = current_value;
                    ee_register_change_timers[i] = 1.0f; // Set timer to 1 second
                }
                ImVec4 color = ImVec4(1.0f, 1.0f - ee_register_change_timers[i], 1.0f - ee_register_change_timers[i], 1.0f);
                ImGui::PushStyleColor(ImGuiCol_Text, color);
                ImGui::Text("%-3s: 0x%016llX%016llX", mips_register_names[i].c_str(), current_value.u64[1], current_value.u64[0]);
                ImGui::PopStyleColor();
                if (ee_register_change_timers[i] > 0.0f) {
                    ee_register_change_timers[i] -= ImGui::GetIO().DeltaTime;
                }
            }
        } else if (auto* iop = dynamic_cast<IOP*>(cpu)) {
            for (int i = 0; i < 32; ++i) {
                ImGui::TableNextColumn();
                std::uint32_t current_value = iop->registers[i];
                if (previous_iop_registers[i] != current_value) {
                    previous_iop_registers[i] = current_value;
                    iop_register_change_timers[i] = 1.0f; // Set timer to 1 second
                }
                ImVec4 color = ImVec4(1.0f, 1.0f - iop_register_change_timers[i], 1.0f - iop_register_change_timers[i], 1.0f);
                ImGui::PushStyleColor(ImGuiCol_Text, color);
                ImGui::Text("%-3s: 0x%08X", mips_register_names[i].c_str(), current_value);
                ImGui::PopStyleColor();
                if (iop_register_change_timers[i] > 0.0f) {
                    iop_register_change_timers[i] -= ImGui::GetIO().DeltaTime;
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

void ImGuiDebug::render_jit_blocks(const char* window_name, CPU& cpu) {
    ImGui::BeginChild(window_name, ImVec2(0, ImGui::GetContentRegionAvail().y), true, ImGuiWindowFlags_HorizontalScrollbar);

    ImGui::Text("%s JIT Compiled Blocks:", window_name);
    ImGui::Separator();

    static std::unordered_map<uint32_t, bool> block_open_state;

    const std::unordered_map<uint32_t, CompiledBlock>* block_cache = cpu.get_block_cache();
    if (block_cache && ImGui::BeginTable("jit_table", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Start PC");
        ImGui::TableSetupColumn("End PC");
        ImGui::TableSetupColumn("Code Pointer");
        ImGui::TableSetupColumn("Last Used");
        ImGui::TableSetupColumn("Contains Branch");
        ImGui::TableSetupColumn("LLVM IR");
        ImGui::TableHeadersRow();

        for (const auto& [pc, block] : *block_cache) {
            ImGui::TableNextColumn();
            ImGui::Text("0x%08X", block.start_pc);
            ImGui::TableNextColumn();
            ImGui::Text("0x%08X", block.end_pc);
            ImGui::TableNextColumn();
            ImGui::Text("%p", block.code_ptr);
            ImGui::TableNextColumn();
            ImGui::Text("%llu", block.last_used);
            ImGui::TableNextColumn();
            ImGui::Text("%s", block.contains_branch ? "Yes" : "No");

            // Add a button to expand and show LLVM IR
            ImGui::TableNextColumn();
            if (block_open_state[pc]) {
                if (ImGui::Button(("Hide IR##" + std::to_string(pc)).c_str())) {
                    block_open_state[pc] = false;
                }
                ImGui::Text("%s", block.llvm_ir.c_str());
            } else {
                if (ImGui::Button(("Show IR##" + std::to_string(pc)).c_str())) {
                    block_open_state[pc] = true;
                }
            }
        }

        ImGui::EndTable();
    }

    ImGui::EndChild();
}

std::unordered_map<int, float> tlb_entry_change_timers;
std::unordered_map<int, TLBEntry> previous_tlb_entries;

void ImGuiDebug::render_bus_info(const char* window_name, const Bus& bus) {
    ImGui::Text("TLB Mappings");

    // Start table with 5 columns: entry_hi, entry_lo0, entry_lo1, page_mask, global
    if (ImGui::BeginTable("tlb_table", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        // Define headers for each column
        ImGui::TableSetupColumn("Entry Hi", ImGuiTableColumnFlags_DefaultSort);
        ImGui::TableSetupColumn("Entry Lo0");
        ImGui::TableSetupColumn("Entry Lo1");
        ImGui::TableSetupColumn("Page Mask");
        ImGui::TableSetupColumn("Global");

        // Actually render the header row
        ImGui::TableHeadersRow();

        // Iterate through each TLB entry
        const auto& tlb_entries = bus.tlb.get_tlb_entries();
        for (size_t i = 0; i < tlb_entries.size(); ++i) {
            const TLBEntry& current_entry = tlb_entries[i];

            // Check for changes and update the timer
            if (previous_tlb_entries.find(i) == previous_tlb_entries.end() || previous_tlb_entries[i] != current_entry) {
                previous_tlb_entries[i] = current_entry;
                tlb_entry_change_timers[i] = 1.0f; // Set the timer to 1 second when the entry changes
            }

            // Apply the red highlighting if there was a change
            ImVec4 color = ImVec4(1.0f, 1.0f - tlb_entry_change_timers[i], 1.0f - tlb_entry_change_timers[i], 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Text, color);

            // Display the TLB entry data in the table
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("0x%08X", current_entry.entry_hi);

            ImGui::TableNextColumn();
            ImGui::Text("0x%08X", current_entry.entry_lo0);

            ImGui::TableNextColumn();
            ImGui::Text("0x%08X", current_entry.entry_lo1);

            ImGui::TableNextColumn();
            ImGui::Text("0x%08X", current_entry.page_mask);

            ImGui::TableNextColumn();
            ImGui::Text(current_entry.global ? "Yes" : "No");

            // Pop style color after the entry is displayed
            ImGui::PopStyleColor();

            // Decrease the timer
            if (tlb_entry_change_timers[i] > 0.0f) {
                tlb_entry_change_timers[i] -= ImGui::GetIO().DeltaTime;
            }
        }

        ImGui::EndTable();
    }
}
