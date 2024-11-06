#include <log/log_imgui.hh>
#include <sstream>

void ImGuiLogBackend::log(const std::string &message, LogLevel level)
{
    std::lock_guard<std::mutex> lock(log_mutex);

    std::string parsed_message = message;
    ImVec4 fg_color, bg_color;
    parse_colors_from_message(parsed_message, level, fg_color, bg_color);

    log_entries.push_back({level, parsed_message, fg_color, bg_color, false});

    if (log_entries.size() > 1000) {
        log_entries.pop_front();
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

    ImGui::Begin("Logger");

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

    ImGui::End();
}
