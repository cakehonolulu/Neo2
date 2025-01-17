#include <bus/bus.hh>
#include <ee/ee.hh>
#include <iop/iop.hh>
#include <iostream>
#include <neo2.hh>
#include <string>
#include <vector>
#include <log/log.hh>
#include <log/log_term.hh>
#include <frontends/imgui/imgui_neo2.h>
#include <cpu/breakpoint.hh>

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
#include <argparse/argparse.hpp>

bool logging_enabled = true; // Flag to track logging state
Disassembler disassembler;

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

const std::string regs[32] = {
    "zr", "at", "v0", "v1", "a0", "a1", "a2", "a3",
    "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
    "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
    "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"
};

void log_instruction(EE* cpu) {
    if (!logging_enabled) {
        return;
    }

    std::ofstream logfile("instruction_trace.txt", std::ios::app);

    uint32_t pc = cpu->pc;
    uint32_t opcode = cpu->bus->read32(pc);
    DisassemblyData data = disassembler.disassemble(cpu, pc, opcode, false);

    //logfile << "PC: " << format("{:08X}", cpu->pc) << " " << data.mnemonic << "\n";
    logfile << "PC: " << format("{:08X}", cpu->pc) << "\n";
    logfile << "GPR:\n";
    for (size_t i = 0; i < 32; i++) {
        //if (auto* ee = dynamic_cast<const EE*>(cpu)) {
            logfile << regs[i] << ": " << format("{:016X}", cpu->registers[i].u64[1])
            << format("{:016X} ", cpu->registers[i].u64[0]);
            if ((i - 1) % 2 == 0) logfile << "\n";
        /*} else if (auto* iop = dynamic_cast<const IOP*>(cpu)) {
            logfile << std::hex << iop->registers[i] << " ";
        }*/
    }
    logfile << "\n";
    //if (auto* ee = dynamic_cast<const EE*>(cpu)) {
        logfile << "HI: " << format("{:08X}", cpu->hi.u32[0]) << " LO: " << format("{:08X}", cpu->lo.u32[0]) << "\n";
    //}
    logfile << "------------------------------------------------------------\n";
    logfile.close();
}

void ImGui_Neo2::run(int argc, char **argv)
{
    // Main loop
    bool done = false;
    static bool suppress_exit_notification = false;

    // Backend selection state
    static bool use_jit_ee = true; // Default to JIT
    static bool use_jit_iop = true; // Default to JIT

    // Debug window visibility state
    static bool show_ee_debug = false;
    static bool show_iop_debug = false;

    static bool show_ee_llvm_blocks = false;
    static bool show_iop_llvm_blocks = false;

    static bool show_bus_debug = false;

    static bool show_ram_view = false;

    argparse::ArgumentParser program("Neo2");

    std::atomic<bool> is_running(false);
    std::atomic<bool> stop_requested(false);  // Flag to stop the thread
    std::mutex status_mutex;  // Mutex for thread-safe status updates

    program.add_argument("--bios")
        .help("Path to BIOS")
        .nargs(1);

    program.add_argument("--full-debug")
        .help("Enable debug windows")
        .nargs(0);

    try
    {
        program.parse_args(argc, argv);
    }
    catch (const std::runtime_error &err)
    {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        Neo2::exit(1, Neo2::Subsystem::Frontend);
    }

    if (program.is_used("--bios"))
    {
        std::string bios_path = program.get<std::string>("--bios");
        bus.load_bios(bios_path);
    }

    if (program.is_used("--full-debug"))
    {
        show_ee_debug = true;
        show_iop_debug = true;
    }

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
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking
    io.FontGlobalScale = 1.25f;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    ImGuiDebug debug_interface(*this, this->disassembler);
    Breakpoint breakpoints;
	std::string bios_file_path;
    std::string status_text = "Idle";
    ImVec4 status_color = ImVec4(1.0f, 0.5f, 0.0f, 1.0f); // Orange for Idle

    this->ee.set_backend(use_jit_ee ? EmulationMode::JIT : EmulationMode::Interpreter);
    this->iop.set_backend(use_jit_iop ? EmulationMode::JIT : EmulationMode::Interpreter);

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
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::DockSpaceOverViewport(0, viewport);

        // Menu Bar with Debug options
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Open BIOS")) {
                    ImGuiFileDialog::Instance()->OpenDialog("ChooseBiosFile", "Select BIOS File", ".bin,.rom");
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Debug")) {
                ImGui::MenuItem("Show EE Debugger", nullptr, &show_ee_debug);
                ImGui::MenuItem("Show IOP Debugger", nullptr, &show_iop_debug);
                if (use_jit_ee) {
                    ImGui::MenuItem("Show EE LLVM Blocks", nullptr, &show_ee_llvm_blocks);
                }
                if (use_jit_iop) {
                    ImGui::MenuItem("Show IOP LLVM Blocks", nullptr, &show_iop_llvm_blocks);
                }

                ImGui::MenuItem("Show Bus Debug", nullptr, &show_bus_debug);
                ImGui::MenuItem("Show RAM View", nullptr, &show_ram_view);
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Backend")) {
                if (ImGui::MenuItem("Use JIT for EE", nullptr, &use_jit_ee)) {
                    this->ee.set_backend(use_jit_ee ? EmulationMode::JIT : EmulationMode::Interpreter);
                }
                if (ImGui::MenuItem("Use JIT for IOP", nullptr, &use_jit_iop)) {
                    this->iop.set_backend(use_jit_iop ? EmulationMode::JIT : EmulationMode::Interpreter);
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        // Render the EE debug window
        if (show_ee_debug) {
            ImGui::Begin("EE Debug");

            if (Neo2::is_aborted()) {
                if (ImGui::Button("Reset")) {
                    this->ee.reset();
                    this->iop.reset();
                    Neo2::reset_aborted();
                    suppress_exit_notification = false;
                }
            }

            debug_interface.render_debug_window("EE Debug", &this->ee, debug_interface.use_pseudos_ee, debug_interface.scroll_offset_ee, &breakpoints);

            ImGui::End();
        }

        // Render the IOP debug window
        if (show_iop_debug) {
            ImGui::Begin("IOP Debug");

            if (Neo2::is_aborted()) {
                if (ImGui::Button("Reset")) {
                    this->ee.reset();
                    this->iop.reset();
                    Neo2::reset_aborted();
                    suppress_exit_notification = false;
                }
            }

            debug_interface.render_debug_window("IOP Debug", &this->iop, debug_interface.use_pseudos_iop, debug_interface.scroll_offset_iop, &breakpoints);

            ImGui::End();
        }

        if (show_ee_llvm_blocks) {
            debug_interface.render_jit_blocks("EE LLVM Blocks", this->ee);
        }

        if (show_iop_llvm_blocks) {
            debug_interface.render_jit_blocks("IOP LLVM Blocks", this->iop);
        }

        if (show_bus_debug) {
            debug_interface.render_bus_info("Bus Info", bus);
        }

        if (show_ram_view) {
            debug_interface.render_memory_view(&bus);
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

        {
            // GUI logic for updating status and handling button clicks
            std::lock_guard<std::mutex> lock(status_mutex);
            ImGui::SetNextWindowPos(ImVec2(0, io.DisplaySize.y - ImGui::GetFrameHeight() - 8), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, ImGui::GetFrameHeight()), ImGuiCond_Always);
            ImGui::Begin("Status Bar", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar);
            ImGui::Text("Status: ");
            ImGui::SameLine();
            ImGui::TextColored(status_color, "%s", status_text.c_str());
            ImGui::SameLine();
            ImGui::Text("|");
            ImGui::SameLine();

            // Align buttons with text
            float text_height = ImGui::GetTextLineHeight();
            float button_height = ImGui::GetFrameHeight();
            float vertical_offset = (text_height - button_height) / 2.0f;
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + vertical_offset);

            // Remove button background unless selected
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.2f, 0.2f, 0.5f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.3f, 0.3f, 0.3f, 0.5f));

            if (ImGui::Button(is_running ? "Stop" : "Play")) {
                if (is_running) {
                    // Stop the emulation
                    status_text = "Stopping";
                    status_color = ImVec4(1.0f, 0.0f, 0.0f, 1.0f); // Red for stopping
                    stop_requested = true;  // Set the flag to stop the emulation
                } else {
                    if (breakpoints.is_breakpoint_hit())
                    {
                        breakpoints.clear_breakpoint_notification();
                        stop_requested = false;
                        Neo2::resume_emulation();
                        is_running = true;
                    }

                    // Start the emulation
                    status_text = "Running";
                    status_color = ImVec4(0.0f, 1.0f, 0.0f, 1.0f); // Green for running
                    stop_requested = false; // Reset stop flag

                    // Start the emulation loop in separate threads
                    std::thread ee_thread([this, &is_running, &breakpoints, &stop_requested, &status_text, &status_color]() {
                        while (!stop_requested) {
                            this->ee.run(&breakpoints);
                        }

                        // After breakpoint or stopping, reset the status
                        status_text = "Idle";
                        status_color = ImVec4(1.0f, 0.5f, 0.0f, 1.0f); // Orange for idle
                        is_running = false; // Emulation stopped
                    });

                    // Detach the thread to allow it to run independently
                    ee_thread.detach();
                    is_running = true;
                }
            }


            // Check for breakpoint hit
            if (breakpoints.is_breakpoint_hit()) {
                if (Neo2::is_emulation_paused())
                {
                    status_text = "Breakpoint Hit";
                    status_color = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
                    is_running = false;
                    stop_requested = true;
                }
            }

            // Disable the Step button when running
            if (is_running) {
                ImGui::BeginDisabled();  // Disables any widget between this call and EndDisabled
            }

            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            ImVec2 p = ImGui::GetCursorScreenPos();
            ImVec2 center = ImVec2(p.x + 15, p.y + 15);
            draw_list->AddTriangleFilled(ImVec2(center.x - 7, center.y - 7), ImVec2(center.x - 7, center.y + 7), ImVec2(center.x + 7, center.y), IM_COL32(255, 255, 255, 255));

            ImGui::SameLine();

            text_height = ImGui::GetTextLineHeight();
            button_height = ImGui::GetFrameHeight();
            vertical_offset = (text_height - button_height) / 2.0f;
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + vertical_offset);

            if (ImGui::Button("Step")) {
                if (breakpoints.is_breakpoint_hit()) breakpoints.clear_breakpoint_notification();
                Neo2::resume_emulation();

                status_text = "Stepping";
                status_color = ImVec4(0.0f, 0.0f, 1.0f, 1.0f);
                if (!Neo2::is_aborted()) {
                    this->ee.step();
                    this->iop.step();
                }
                status_text = "Idle";
                status_color = ImVec4(1.0f, 0.5f, 0.0f, 1.0f);
            }

            if (is_running) {
                ImGui::EndDisabled();  // Re-enable widgets after this call
            }

            ImGui::PopStyleColor(3);
            ImGui::End();
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
