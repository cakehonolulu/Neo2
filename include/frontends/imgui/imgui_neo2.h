#pragma once

#include <neo2.hh>
#include "log/log_imgui.hh"

class ImGui_Neo2 : public Neo2
{
  public:
	ImGui_Neo2();
    void init() override;
    void run() override;

  private:
	std::shared_ptr<ImGuiLogBackend> imgui_logger;
};