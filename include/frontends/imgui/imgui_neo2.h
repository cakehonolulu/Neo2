#pragma once

#include <neo2.hh>
#include <thread>
#include "log/log_imgui.hh"

struct PerformanceMetrics
{
    double ee_mips;  // Emulated MIPS
    double ee_usage; // EE usage percentage
    double gs_fps;   // GS frames per second
    double gs_usage; // GS usage percentage (fraction of frame time spent drawing)
};

class ImGui_Neo2 : public Neo2
{
  public:
	  ImGui_Neo2();
    void init() override;
    void run(int argc, char **argv) override;

    void render_fbgl(GS& gs);

    std::thread ee_thread;
    
    int prev_fb_width = 0;
    int prev_fb_height = 0;

    PerformanceMetrics gPerfMetrics = {0.0, 0.0, 0.0, 0.0};
    
  private:
	  std::shared_ptr<ImGuiLogBackend> imgui_logger;
};