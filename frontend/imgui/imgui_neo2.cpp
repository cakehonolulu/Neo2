#include <bus/bus.hh>
#include <ee/ee.hh>
#include <iostream>
#include <neo2.hh>
#include <string>
#include <vector>
#include <log/log.hh>
#include <log/log_term.hh>
#include <frontends/imgui/imgui_neo2.h>

#include "frontends/imgui/imgui_debug.hh"
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlrenderer3.h"
#include "log/log_imgui.hh"
#include <SDL3/SDL.h>
#include <stdio.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL3/SDL_opengles2.h>
#else
#include <SDL3/SDL_opengl.h>
#endif
#include "ImGuiFileDialog.h"

ImGui_Neo2::ImGui_Neo2()
    : Neo2(std::make_shared<ImGuiLogBackend>())
{
    imgui_logger = std::dynamic_pointer_cast<ImGuiLogBackend>(Logger::get_backends().back());
    Logger::debug("Logger initialized\n");
}

void ImGui_Neo2::init()
{
    Logger::debug("Initializing ImGui frontend...\n");
}

void ImGui_Neo2::run()
{
    // Setup SDL
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD))
    {
        printf("Error: SDL_Init(): %s\n", SDL_GetError());
        return;
    }

    // Create window with SDL_Renderer graphics context
    Uint32 window_flags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN;
    SDL_Window* window = SDL_CreateWindow("Neo2 - ImGui + SDL3", 1280, 720, window_flags);
    if (window == nullptr)
    {
        printf("Error: SDL_CreateWindow(): %s\n", SDL_GetError());
        return;
    }
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    SDL_SetRenderVSync(renderer, 1);
    if (renderer == nullptr)
    {
        SDL_Log("Error: SDL_CreateRenderer(): %s\n", SDL_GetError());
        return;
    }
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    SDL_ShowWindow(window);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    bool done = false;
    bool show_ee_disassembler = false;
    bool show_iop_disassembler = false;
    bool show_ee_registers = false;
    bool show_iop_registers = false;
    static bool suppress_exit_notification = false;

    ImGuiDebug debug_interface(*this, this->disassembler);
	std::string bios_file_path;
#ifdef __EMSCRIPTEN__
    // For an Emscripten build we are disabling file-system access, so let's not attempt to do a fopen() of the imgui.ini file.
    // You may manually call LoadIniSettingsFromMemory() to load settings from your own storage.
    io.IniFilename = nullptr;
    EMSCRIPTEN_MAINLOOP_BEGIN
#else
    while (!done)
#endif
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT)
                done = true;
            if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(window))
                done = true;
        }
        if (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED)
        {
            SDL_Delay(10);
            continue;
        }

        // Start the Dear ImGui frame
        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        // Menu Bar with Debug options
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Open BIOS")) {
                    ImGuiFileDialog::Instance()->OpenDialog("ChooseBiosFile", "Select BIOS File", ".bin,.rom");
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Debug")) {
                ImGui::MenuItem("EE Disassembly", nullptr, &show_ee_disassembler);
                ImGui::MenuItem("IOP Disassembly", nullptr, &show_iop_disassembler);
                ImGui::MenuItem("EE Registers", nullptr, &show_ee_registers);
                ImGui::MenuItem("IOP Registers", nullptr, &show_iop_registers);
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        // Render disassembly windows if toggled on
        if (show_ee_disassembler) {
            debug_interface.render_cpu_disassembly("EE Disassembler", this->ee.pc, &this->ee, debug_interface.use_pseudos_ee, debug_interface.scroll_offset_ee);
        }
        if (show_iop_disassembler) {
            debug_interface.render_cpu_disassembly("IOP Disassembler", this->iop.pc, &this->iop, debug_interface.use_pseudos_iop, debug_interface.scroll_offset_iop);
        }
        if (show_ee_registers) {
            debug_interface.render_cpu_registers("EE Registers", &this->ee);
        }
        if (show_iop_registers) {
            debug_interface.render_cpu_registers("IOP Registers", &this->iop);
        }

        // Handle the file dialog for loading BIOS
        if (ImGuiFileDialog::Instance()->Display("ChooseBiosFile")) {
            if (ImGuiFileDialog::Instance()->IsOk()) {
                bios_file_path = ImGuiFileDialog::Instance()->GetFilePathName();
				std::string log_message = "BIOS file selected: " + bios_file_path;
                Logger::info(log_message);

				bus.load_bios(bios_file_path);
            }
            ImGuiFileDialog::Instance()->Close();
        }

		imgui_logger->render();

        if (Neo2::is_aborted() && !suppress_exit_notification) {
            // Set window size and open popup
            ImGui::SetNextWindowSize(ImVec2(350, 150), ImGuiCond_Appearing);
            ImGui::OpenPopup("Exit Notification");

            bool open = true;
            if (ImGui::Begin("Exit Notification", &open, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
                ImGui::Text("The emulator encountered a critical error.");

                // Reset button to clear the aborted state and resume
                if (ImGui::Button("Reset")) {
                    this->ee.reset();
                    this->iop.reset();
                    Neo2::reset_aborted();
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();

                // Close button to exit the emulator
                if (ImGui::Button("Exit emulator")) {
                    done = true;
                    ImGui::CloseCurrentPopup();
                }

                ImGui::End();
            }

            // If "X" button or other close action is taken, suppress the popup for now
            if (!open) {
                suppress_exit_notification = true;
            }
        }

        // Reset suppress_exit_notification if aborted state is cleared
        if (!Neo2::is_aborted()) {
            suppress_exit_notification = false;
        }

        // Rendering
        ImGui::Render();
        //SDL_RenderSetScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
        SDL_SetRenderDrawColorFloat(renderer, clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }
#ifdef __EMSCRIPTEN__
    EMSCRIPTEN_MAINLOOP_END;
#endif

    // Cleanup
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
