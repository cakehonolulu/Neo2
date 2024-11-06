#pragma once

#include <log/log.hh>
#include <imgui.h>
#include <deque>
#include <string>
#include <mutex>

class ImGuiLogBackend : public LogBackend
{
  public:
    void log(const std::string &message, LogLevel level) override;
    void render();

  void raw_special(const std::string& message, LogLevel level);private:
    struct LogEntry {
        LogLevel level;
        std::string message;
        ImVec4 fg_color;  // Foreground color
        ImVec4 bg_color;  // Background color
        bool special;     // Flag to mark special logs like banners
    };

    std::deque<LogEntry> log_entries;
    std::mutex log_mutex;

    static ImVec4 get_color_for_level(LogLevel level);
    static void parse_colors_from_message(std::string& message, LogLevel level, ImVec4& fg_color, ImVec4& bg_color);
};
