#pragma once

#include <neo2.hh>
#include <log/log_imgui.hh>
#include <string>
#include <imgui.h>

#include <thread>
#include <atomic>
#include <mutex>

class ImGui_Neo2 : public Neo2
{
public:
    ImGui_Neo2();
    void init() override;
    void run() override;

private:
    std::shared_ptr<ImGuiLogBackend> imgui_logger;
    
    std::atomic<bool> stop_requested;  // Flag to stop the thread
    std::mutex status_mutex;          // Mutex for thread-safe status updates
    std::string status_text;          // Current status text
    ImVec4 status_color;              // Current status color
};
