#include <log/log_imgui.hh>
#include <sstream>

void ImGuiLogBackend::log(const std::string &message, LogLevel level)
{
    std::lock_guard<std::mutex> lock(log_mutex);

    std::string parsed_message = message;
    ImVec4 fg_color, bg_color;
    parse_colors_from_message(parsed_message, level, fg_color, bg_color);

    if (level == LogLevel::EE_LOG) {
        ee_log_entries.push_back({level, parsed_message, fg_color, bg_color, false});
    } else {
        log_entries.push_back({level, parsed_message, fg_color, bg_color, false});
    }

    // Optional: Limit size of the log buffers
    if (log_entries.size() > 1000) {
        log_entries.pop_front();
    }

    if (ee_log_entries.size() > 100) {
        ee_log_entries.pop_front();  // Limit EE log buffer size
    }
}

void ImGuiLogBackend::raw_special(const std::string &message, LogLevel level)
{
    std::lock_guard<std::mutex> lock(log_mutex);

    std::string parsed_message = message;
    ImVec4 fg_color, bg_color;
    parse_colors_from_message(parsed_message, level, fg_color, bg_color);

    log_entries.push_back({level, parsed_message, fg_color, bg_color, true});

    if (log_entries.size() > 1000) {
        log_entries.pop_front();
    }
}

void ImGuiLogBackend::parse_colors_from_message(std::string& message, LogLevel level, ImVec4& fg_color, ImVec4& bg_color)
{
    fg_color = get_color_for_level(level);
    bg_color = ImVec4(0, 0, 0, 0); // Default transparent background

    size_t color_start = message.find("[fg_color(");
    if (color_start != std::string::npos) {
        size_t color_end = message.find("),", color_start);
        if (color_end != std::string::npos) {
            std::string color_substring = message.substr(color_start + 10, color_end - (color_start + 10));
            std::istringstream color_stream(color_substring);
            int r, g, b;
            char comma;

            if (color_stream >> r >> comma >> g >> comma >> b) {
                fg_color = ImVec4(r / 255.0f, g / 255.0f, b / 255.0f, 1.0f);
                message.erase(color_start, color_end - color_start + 2);
            }
        }
    }

    size_t bg_start = message.find("bg_color(");
    if (bg_start != std::string::npos) {
        size_t bg_end = message.find(")]", bg_start);
        if (bg_end != std::string::npos) {
            std::string bg_color_substring = message.substr(bg_start + 9, bg_end - (bg_start + 9));
            std::istringstream bg_color_stream(bg_color_substring);
            int r, g, b;
            char comma;

            if (bg_color_stream >> r >> comma >> g >> comma >> b) {
                bg_color = ImVec4(r / 255.0f, g / 255.0f, b / 255.0f, 1.0f);
                message.erase(bg_start, bg_end - bg_start + 2);
            }
        }
    }
}

ImVec4 ImGuiLogBackend::get_color_for_level(LogLevel level)
{
    switch (level)
    {
    case LogLevel::Info:
        return ImVec4(1.0f, 1.0f, 1.0f, 1.0f); // White
    case LogLevel::Warn:
        return ImVec4(1.0f, 1.0f, 0.0f, 1.0f); // Yellow
    case LogLevel::Debug:
        return ImVec4(0.0f, 1.0f, 1.0f, 1.0f); // Cyan
    case LogLevel::Error:
        return ImVec4(1.0f, 0.0f, 0.0f, 1.0f); // Red
    default:
        return ImVec4(1.0f, 1.0f, 1.0f, 1.0f); // Default white
    }
}

void ImGuiLogBackend::render()
{
    std::lock_guard<std::mutex> lock(log_mutex);

    static bool selectable_view = false;

    ImGui::Begin("Logger");

    if (selectable_view) {
        std::string combined_log;
        for (const auto &entry : log_entries) {
            combined_log += entry.message + "\n";
        }
        ImGui::InputTextMultiline("##log_entries", &combined_log[0], combined_log.size() + 1, ImVec2(-FLT_MIN, ImGui::GetTextLineHeight() * 16), ImGuiInputTextFlags_ReadOnly);
    } else {
        for (const auto &entry : log_entries) {
            if (entry.special) {
                ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
            }

            if (entry.bg_color.w > 0) {
                ImVec2 text_size = ImGui::CalcTextSize(entry.message.c_str());
                ImVec2 cursor_pos = ImGui::GetCursorScreenPos();

                ImGui::GetWindowDrawList()->AddRectFilled(
                    cursor_pos,
                    ImVec2(cursor_pos.x + text_size.x, cursor_pos.y + text_size.y),
                    ImGui::ColorConvertFloat4ToU32(entry.bg_color)
                );
            }

            ImGui::PushStyleColor(ImGuiCol_Text, entry.fg_color);
            ImGui::TextUnformatted(entry.message.c_str());
            ImGui::PopStyleColor();

            if (entry.special) {
                ImGui::PopStyleVar();
            }
        }
    }

    ImVec2 window_size = ImGui::GetWindowSize();
    ImVec2 button_size = ImGui::CalcTextSize(selectable_view ? "Rich text" : "Selectable logs");
    button_size.x += ImGui::GetStyle().FramePadding.x * 2.0f;
    button_size.y += ImGui::GetStyle().FramePadding.y * 2.0f;

    ImGui::SetCursorPos(ImVec2(window_size.x - button_size.x - ImGui::GetStyle().ItemSpacing.x, window_size.y - button_size.y - ImGui::GetStyle().ItemSpacing.y - ImGui::GetStyle().WindowPadding.y));
    if (ImGui::Button(selectable_view ? "Rich text" : "Selectable logs", button_size)) {
        selectable_view = !selectable_view;
    }

    if (ImGui::Button("Clear", button_size)) {
        log_entries.clear();
    }

    ImGui::End();

    ImGui::Begin("EE Logs");

    if (selectable_view) {
        std::string combined_ee_log;
        for (const auto &entry : ee_log_entries) {
            combined_ee_log += entry.message + "\n";
        }
        ImGui::InputTextMultiline("##ee_log_entries", &combined_ee_log[0], combined_ee_log.size() + 1, ImVec2(-FLT_MIN, ImGui::GetTextLineHeight() * 16), ImGuiInputTextFlags_ReadOnly);
    } else {
        for (const auto &entry : ee_log_entries) {
            if (entry.special) {
                ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
            }

            if (entry.bg_color.w > 0) {
                ImVec2 text_size = ImGui::CalcTextSize(entry.message.c_str());
                ImVec2 cursor_pos = ImGui::GetCursorScreenPos();

                ImGui::GetWindowDrawList()->AddRectFilled(
                    cursor_pos,
                    ImVec2(cursor_pos.x + text_size.x, cursor_pos.y + text_size.y),
                    ImGui::ColorConvertFloat4ToU32(entry.bg_color)
                );
            }

            ImGui::PushStyleColor(ImGuiCol_Text, entry.fg_color);
            ImGui::TextUnformatted(entry.message.c_str());
            ImGui::PopStyleColor();

            if (entry.special) {
                ImGui::PopStyleVar();
            }
        }
    }

    window_size = ImGui::GetWindowSize();
    button_size = ImGui::CalcTextSize(selectable_view ? "Rich text" : "Selectable logs");
    button_size.x += ImGui::GetStyle().FramePadding.x * 2.0f;
    button_size.y += ImGui::GetStyle().FramePadding.y * 2.0f;

    ImGui::SetCursorPos(ImVec2(window_size.x - button_size.x - ImGui::GetStyle().ItemSpacing.x, window_size.y - button_size.y - ImGui::GetStyle().ItemSpacing.y - ImGui::GetStyle().WindowPadding.y));
    if (ImGui::Button(selectable_view ? "Rich text" : "Selectable logs", button_size)) {
        selectable_view = !selectable_view;
    }

    if (ImGui::Button("Clear", button_size)) {
        ee_log_entries.clear();
    }

    ImGui::End();
}
