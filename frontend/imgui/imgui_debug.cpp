#include "imgui.h"
#include "imgui_internal.h"
#include <cpu/disassembler.hh>
#include <frontends/imgui/imgui_debug.hh>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include "ImGuiFileDialog.h"

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

void ImGuiDebug::render_memory_view(Bus *bus) {
    ImGui::Begin("RAM View");

    ImGui::Text("RAM View");

    ImGui::SameLine();

    if (ImGui::Button("Save RAM Dump")) {
        ImGuiFileDialog::Instance()->OpenDialog("SaveRAMDump", "Save RAM Dump", ".bin");
    }

    if (ImGuiFileDialog::Instance()->Display("SaveRAMDump")) {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            std::string save_path = ImGuiFileDialog::Instance()->GetFilePathName();
            std::ofstream file(save_path, std::ios::out | std::ios::binary);
            file.write(reinterpret_cast<const char*>(bus->ram.data()), bus->ram.size());
            file.close();
        }
        ImGuiFileDialog::Instance()->Close();
    }

    ImGui::Separator();

    // Textbox to enter a virtual address and call bus map_to_phys to get the phys address
    static char address_buffer[9] = "00000000";

    if (ImGui::InputText("Virtual Address", address_buffer, sizeof(address_buffer), ImGuiInputTextFlags_CharsHexadecimal)) {
        // Ensure the buffer is null-terminated
        address_buffer[8] = '\0';
    }
    
    ImGui::SameLine();

    // Button that calls map_to_phys and displays the physical address
    if (ImGui::Button("Map to Physical Address")) {
        uint32_t vaddr = std::stoul(address_buffer, nullptr, 16);
        uint32_t paddr = bus->map_to_phys(vaddr, bus->tlb);
        Logger::info("Virtual Address 0x" + std::string(address_buffer) + " maps to Physical Address 0x" + format("{:08X}", paddr));
    }

    mem_edit.DrawContents(bus->ram.data(), bus->ram.size());

    ImGui::Separator();
    ImGui::End();
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

void ImGuiDebug::add_breakpoint(uint32_t address, CPU& cpu) {
    if (dynamic_cast<EE*>(&cpu)) {
        ee_breakpoints.insert(address);
    } else if (dynamic_cast<IOP*>(&cpu)) {
        iop_breakpoints.insert(address);
    }
}

void ImGuiDebug::remove_breakpoint(uint32_t address, CPU& cpu) {
    if (dynamic_cast<EE*>(&cpu)) {
        ee_breakpoints.erase(address);
    } else if (dynamic_cast<IOP*>(&cpu)) {
        iop_breakpoints.erase(address);
    }
}

bool ImGuiDebug::has_breakpoint(uint32_t address, CPU& cpu) {
    if (dynamic_cast<EE*>(&cpu)) {
        return ee_breakpoints.count(address) > 0;
    } else if (dynamic_cast<IOP*>(&cpu)) {
        return iop_breakpoints.count(address) > 0;
    }
    return false;
}

void ImGuiDebug::render_debug_window(const char* window_name, CPU* cpu, bool& pseudos, int& scroll_offset) {
    ImGui::Begin(window_name);  // Start the combined window

    // Create a layout with 2 columns (Disassembly and Registers)
    ImGui::Columns(2, nullptr, false); // Two columns, no border

    // Set the width of the first column (Disassembly)
    ImGui::SetColumnWidth(0, 400); // Set the disassembly column to a fixed width

    // Disassembly Section - Left Column
    ImGui::BeginGroup();  // Start a new group for the Disassembly Section
    ImGui::Text("Disassembly");

    ImGui::SeparatorEx(ImGuiSeparatorFlags_Horizontal);
    ImGui::Checkbox("Use Pseudocodes", &pseudos);
    
    // Begin a child window for disassembly view
    ImGui::BeginChild("DisassemblyView", ImVec2(0, ImGui::GetContentRegionAvail().y), true, ImGuiWindowFlags_HorizontalScrollbar);
    const float line_height = ImGui::GetTextLineHeightWithSpacing();
    const int instructions_per_page = static_cast<int>(ImGui::GetContentRegionAvail().y / line_height);

    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)) {
        scroll_offset += static_cast<int>(ImGui::GetIO().MouseWheel * -1);
    }

    uint32_t display_pc = (dynamic_cast<EE*>(cpu) ? dynamic_cast<EE*>(cpu)->pc : dynamic_cast<IOP*>(cpu)->pc);
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
    ImGui::EndGroup();  // End Disassembly Section

    ImGui::NextColumn(); // Move to the next column (Registers Section)

    // Registers Section - Right Column
    ImGui::BeginGroup();  // Start a new group for Registers Section
    ImGui::Text("Registers");
    ImGui::SeparatorEx(ImGuiSeparatorFlags_Horizontal);

    // Variables to hold the selected register and its new value
    static int selected_register = -1;
    static char new_value[128] = "";

    // Lambda to render register with optional input field
    auto render_register_ = [&](const char* label, __int128 value, int reg_index = -1) {
        ImGui::Text("%s: 0x%032llX", label, value);
        if (ImGui::IsItemClicked()) {
            selected_register = reg_index;
            snprintf(new_value, sizeof(new_value), "%032llX", value);
        }

        if (selected_register == reg_index) {
            ImGui::InputText("##edit", new_value, sizeof(new_value), ImGuiInputTextFlags_CharsHexadecimal);
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                value = strtoull(new_value, nullptr, 16);
                if (auto* ee = dynamic_cast<EE*>(cpu)) {
                    ee->registers[reg_index].u128 = value;
                }
                selected_register = -1; // Deselect the register after editing
            }
        }
    };

    // Lambda to render register with optional input field
    auto render_register = [&](const char* label, uint64_t value, int reg_index = -1) {
        ImGui::Text("%s: 0x%016llX", label, value);
        if (ImGui::IsItemClicked()) {
            selected_register = reg_index;
            snprintf(new_value, sizeof(new_value), "%016llX", value);
        }

        if (selected_register == reg_index) {
            ImGui::InputText("##edit", new_value, sizeof(new_value), ImGuiInputTextFlags_CharsHexadecimal);
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                value = strtoull(new_value, nullptr, 16);
                if (auto* ee = dynamic_cast<EE*>(cpu)) {
                    ee->registers[reg_index].u128 = value;
                } else if (auto* iop = dynamic_cast<IOP*>(cpu)) {
                    iop->registers[reg_index] = value;
                }
                selected_register = -1; // Deselect the register after editing
            }
        }
    };

    if (auto* ee = dynamic_cast<EE*>(cpu)) {
        ImGui::Text("EE PC: 0x%08X (%08X)", ee->pc, ee->next_pc);
        ImGui::Text("LO: 0x%08X", ee->lo);
        ImGui::Text("HI: 0x%08X", ee->hi);
    } else if (auto* iop = dynamic_cast<IOP*>(cpu)) {
        ImGui::Text("IOP PC: 0x%08X (%08X)", iop->pc, iop->next_pc);
    }

    // General-purpose registers table
    if (ImGui::BeginTable("gpr_table", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        if (auto* ee = dynamic_cast<EE*>(cpu)) {
            for (int i = 0; i < 32; ++i) {
                ImGui::TableNextColumn();
                uint128_t current_value = ee->registers[i];
                if (previous_ee_registers[i].u128 != current_value.u128) {
                    previous_ee_registers[i] = current_value;
                    ee_register_change_timers[i] = 1.0f;
                }
                ImVec4 color = ImVec4(1.0f, 1.0f - ee_register_change_timers[i], 1.0f - ee_register_change_timers[i], 1.0f);
                ImGui::PushStyleColor(ImGuiCol_Text, color);
                render_register_(mips_register_names[i].c_str(), current_value.u128, i); // Use the lambda to render registers
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
                    iop_register_change_timers[i] = 1.0f;
                }
                ImVec4 color = ImVec4(1.0f, 1.0f - iop_register_change_timers[i], 1.0f - iop_register_change_timers[i], 1.0f);
                ImGui::PushStyleColor(ImGuiCol_Text, color);
                render_register(mips_register_names[i].c_str(), current_value, i); // Use the lambda to render registers
                                ImGui::PopStyleColor();
                if (iop_register_change_timers[i] > 0.0f) {
                    iop_register_change_timers[i] -= ImGui::GetIO().DeltaTime;
                }
            }
        }
        ImGui::EndTable();
    }

    ImGui::SeparatorEx(ImGuiSeparatorFlags_Horizontal);
    ImGui::Text("COP0 Registers");

    // COP0 registers table for both EE and IOP
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

    ImGui::SeparatorEx(ImGuiSeparatorFlags_Horizontal);
    
    // Breakpoints Section - Below Registers
    ImGui::Text("Breakpoints");
    static uint32_t breakpoint_address = 0;
    ImGui::InputScalar("Breakpoint Address", ImGuiDataType_U32, &breakpoint_address, nullptr, nullptr, "%08X", ImGuiInputTextFlags_CharsHexadecimal);
    
    if (ImGui::Button("Add Breakpoint")) {
        add_breakpoint(breakpoint_address, *cpu);
    }

    ImGui::SameLine();
    
    if (ImGui::Button("Remove Breakpoint")) {
        remove_breakpoint(breakpoint_address, *cpu);
    }

    ImGui::Text("Breakpoints:");
    if (auto* ee = dynamic_cast<EE*>(cpu)) {
        for (const auto& bp : ee_breakpoints) {
            ImGui::BulletText("EE: 0x%08X", bp);
        }
    } else if (auto* iop = dynamic_cast<IOP*>(cpu)) {
        for (const auto& bp : iop_breakpoints) {
            ImGui::BulletText("IOP: 0x%08X", bp);
        }
    }

    ImGui::EndGroup();  // End Registers Section

    ImGui::Columns(1);  // End columns layout (back to one column)

    ImGui::End(); // End the combined window
}
