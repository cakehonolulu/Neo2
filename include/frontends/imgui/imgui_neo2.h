#pragma once

#include <neo2.hh>
#include <thread>
#include "log/log_imgui.hh"

class ImGui_Neo2 : public Neo2
{
  public:
	ImGui_Neo2();
    void init() override;
    void run(int argc, char **argv) override;

  std::thread ee_thread;

  private:
	std::shared_ptr<ImGuiLogBackend> imgui_logger;
};