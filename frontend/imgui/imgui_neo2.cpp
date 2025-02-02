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

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define SERVER_IP "127.0.0.1"
#define PORT 12345

int client_fd = -1;

void connectToPCSX2() {
    struct sockaddr_in server_address;

    // Create the socket
    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(PORT);

    // Convert server IP address
    if (inet_pton(AF_INET, SERVER_IP, &server_address.sin_addr) <= 0) {
        perror("Invalid server address");
        exit(EXIT_FAILURE);
    }

    // Connect to the server
    while (connect(client_fd, (struct sockaddr*)&server_address, sizeof(server_address)) < 0) {
        Logger::info("Waiting for PCSX2 server...");
        sleep(1);
    }

    Logger::info("Connected to PCSX2 server!");
}

#undef Default
#include "json.hpp"
using json = nlohmann::json;

std::string to_hex_string(uint32_t value) {
    std::ostringstream stream;
    stream << std::hex << std::uppercase << value;
    return stream.str();
}

std::string to_hex_string(uint128_t value) {
    uint64_t high = value.u64[1];
    uint64_t low = value.u64[0];
    std::stringstream ss;
    ss << std::hex << std::setw(16) << std::setfill('0') << high
       << std::setw(16) << std::setfill('0') << low;
    return ss.str();
}

static size_t step_counter = 0;  // Sequence number
// Function to compare two JSON objects and highlight differences
void compareAndPrintJsonDiff(const json& client_state, const json& server_state) {
    for (json::const_iterator it = client_state.begin(); it != client_state.end(); ++it) {
        // check if it.key() contains cop0_reg_ and skip
        if (it.key().find("cop0_reg") == 0) {
            continue;
        }
        // If key exists in server state
        if (server_state.contains(it.key())) {
            if (it.value() != server_state[it.key()]) {
                // Color differences red
                std::cout << "\033[31mDifference in key: " << it.key() << "\033[0m" << std::endl;
                std::cout << "  Neo2 state: " << it.value() << std::endl;
                std::cout << "  PCSX2 state: " << server_state[it.key()] << std::endl;
            }
        } else {
            // Key not in server state, print in red
            std::cout << "\033[31mKey missing in server state: " << it.key() << "\033[0m" << std::endl;
        }
    }

    // Check for keys in server state that are missing in client state
    for (json::const_iterator it = server_state.begin(); it != server_state.end(); ++it) {
        // Skip cop0_reg* entries
        if (it.key().find("cop0_reg") == 0) {
            continue;
        }

        if (!client_state.contains(it.key())) {
            // Missing key in client state, print in red
            std::cout << "\033[31mKey missing in client state: " << it.key() << "\033[0m" << std::endl;
        }
    }
}

int compareRegistersWithPCSX2(EE* core) {
    int ret = 1;

    // Step 1: Send the `ready_number` to PCSX2
    std::string ready_signal = "ready_" + std::to_string(step_counter) + "\n";
    send(client_fd, ready_signal.c_str(), ready_signal.size(), 0);

    // Step 2: Wait for the register JSON from PCSX2
    char buffer[10240] = {0};
    size_t total_bytes = 0;

    while (true) {
        int bytes_received = read(client_fd, buffer + total_bytes, sizeof(buffer) - total_bytes - 1);
        if (bytes_received <= 0) {
            std::cerr << "Error: Lost connection to PCSX2." << std::endl;
            exit(1);
        }
        total_bytes += bytes_received;
        buffer[total_bytes] = '\0';  // Ensure null-termination

        if (buffer[total_bytes - 1] == '\n') break;  // End of message detected
    }

    json server_state = json::parse(buffer);
    
    // Get "PC" key from server_state as uint32_t
    uint32_t server_pc = std::stoul(server_state["pc"].get<std::string>(), nullptr, 16);

    // Step 3: Generate Neo2's current state
    json neo2_state = {
        {"pc", "0x" + to_hex_string(core->pc)},
        {"hi", "0x" + to_hex_string(core->hi)},
        {"lo", "0x" + to_hex_string(core->lo)}
    };

    for (int i = 0; i < 32; ++i) {
        neo2_state["reg_" + std::to_string(i)] = "0x" + to_hex_string(core->registers[i]);
    }

    for (int i = 0; i < 32; ++i) {
        neo2_state["cop0_reg_" + std::to_string(i)] = "0x" + to_hex_string(core->cop0.regs[i]);
    }

    // Print client state and server state differences
    compareAndPrintJsonDiff(neo2_state, server_state);

    std::cerr << "Init PC: 0x" << std::hex << core->pc << std::endl;
    // Filter out cop0_reg_* entries from a JSON object
    auto filter_cop0_regs = [](const json& state) -> json {
        json filtered_state;
        for (auto it = state.begin(); it != state.end(); ++it) {
            if (it.key().find("cop0_reg") != 0) { // Skip keys starting with "cop0_reg"
                filtered_state[it.key()] = it.value();
            }
        }
        return filtered_state;
    };

    // Filter the states
    json filtered_neo2_state = filter_cop0_regs(neo2_state);
    json filtered_server_state = filter_cop0_regs(server_state);

    // Compare filtered states
    if (filtered_neo2_state != filtered_server_state) {
        std::cerr << "Mismatch detected!" << std::endl;
        std::cerr << "Neo2 state: " << filtered_neo2_state.dump(4) << std::endl;
        std::cerr << "PCSX2 state: " << filtered_server_state.dump(4) << std::endl;

        // Send `MISMATCH` to PCSX2
        std::string mismatch_signal = "MISMATCH\n";
        send(client_fd, mismatch_signal.c_str(), mismatch_signal.size(), 0);
        exit(1);
    }

    // Step 4: Send `ACK` to PCSX2
    std::string ack_signal = "ACK\n";
    send(client_fd, ack_signal.c_str(), ack_signal.size(), 0);

    // Increment the step counter
    step_counter++;

    ret = 0;
    return ret;
}

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

SDL_Texture* vram_texture = nullptr;
float zoom_factor = 1.0f;


SDL_Texture* create_texture_from_vram(SDL_Renderer* renderer, GS& gs, const GS::Texture& texture) {
    int tex_h = texture.height * 2;
    SDL_Texture* sdl_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_XBGR8888, SDL_TEXTUREACCESS_STREAMING, texture.width, tex_h);
    SDL_SetTextureScaleMode(sdl_texture, SDL_SCALEMODE_LINEAR);

    void* pixels;
    int pitch;
    SDL_LockTexture(sdl_texture, nullptr, &pixels, &pitch);

    uint32_t* dst = static_cast<uint32_t*>(pixels);
    uint32_t* src = reinterpret_cast<uint32_t*>(gs.vram);

    for (uint32_t y = 0; y < texture.width; ++y) {
        for (uint32_t x = 0; x < tex_h; ++x) {
            uint32_t vram_index = (texture.address) + y * tex_h + x;
            dst[y * tex_h + x] = src[vram_index];
        }
    }

    SDL_UnlockTexture(sdl_texture);
    return sdl_texture;
}


void render_textures(SDL_Renderer* renderer, GS& gs) {
    ImGui::Begin("Textures", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    const auto& textures = gs.get_textures();
    static int selected_texture_index = -1;
    static std::vector<bool> texture_windows_open(textures.size(), false);
    static std::vector<SDL_Texture*> sdl_textures(textures.size(), nullptr);

    ImGui::Text("Available Textures:");
    ImGui::Separator();

    for (size_t i = 0; i < textures.size(); ++i) {
        const auto& texture = textures[i];
        if (ImGui::Selectable(texture.name.c_str(), selected_texture_index == static_cast<int>(i))) {
            selected_texture_index = static_cast<int>(i);
            texture_windows_open[i] = true;
            if (!sdl_textures[i]) {
                sdl_textures[i] = create_texture_from_vram(renderer, gs, texture);
            }
        }
    }

    ImGui::Separator();

    for (size_t i = 0; i < textures.size(); ++i) {
        if (texture_windows_open[i]) {
            const auto& texture = textures[i];
            bool open = texture_windows_open[i];
            ImGui::Begin(texture.name.c_str(), &open, ImGuiWindowFlags_AlwaysAutoResize);
            texture_windows_open[i] = open;

            ImGui::Text("Address: 0x%08X", texture.address);
            ImGui::Text("Width: %d", texture.width);
            ImGui::Text("Height: %d", texture.height);

            std::string format;

            switch (texture.format) {
                case 0x00:
                    format="PSMCT32";
                    break;
                case 0x01:
                    format="PSMCT24";
                    break;
                case 0x02:
                    format="PSMCT16";
                    break;
                case 0x0A:
                    format="PSMCT16S";
                    break;
                case 0x13:
                    format="PSMCT8";
                    break;
                case 0x14:
                    format="PSMCT4";
                    break;
                case 0x1B:
                    format="PSMCT8H";
                    break;
                case 0x24:
                    format="PSMCT4HL";
                    break;
                case 0x2C:
                    format="PSMCT4HH";
                    break;
                case 0x30:
                    format="PSMZ32";
                    break;
                case 0x31:
                    format="PSMZ24";
                    break;
                case 0x32:
                    format="PSMZ16";
                    break;
                case 0x3A:
                    format="PSMZ16S";
                    break;
                default:
                    format="Unknown";
                    break;
            }
            ImGui::Text("Format: %s", format.c_str());

            ImVec2 texture_size = ImVec2(texture.width * zoom_factor, texture.height * zoom_factor);
            ImGui::Image(reinterpret_cast<ImTextureID>(sdl_textures[i]), texture_size);

            ImGui::End();

            if (!open) {
                SDL_DestroyTexture(sdl_textures[i]);
                sdl_textures[i] = nullptr;
            }
        }
    }

    ImGui::End();
}


void render_framebuffer(SDL_Renderer* renderer, GS& gs) {
    ImGui::Begin("Framebuffer", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    if (ImGui::BeginTabBar("FramebufferTabs")) {
        if (ImGui::BeginTabItem("FRAME_1")) {
            ImGui::Text("Framebuffer 1 Size: %dx%d", gs.framebuffer1.width, gs.framebuffer1.height);


            float tex_w, tex_h;
            if (!vram_texture || SDL_GetTextureSize(vram_texture, &tex_w, &tex_h) != 0 || tex_w != gs.framebuffer1.width || tex_h != gs.framebuffer1.height) {
                if (vram_texture) {
                    SDL_DestroyTexture(vram_texture);
                }
                vram_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_XBGR8888, SDL_TEXTUREACCESS_STREAMING, gs.framebuffer1.fbw, gs.framebuffer1.height);
                SDL_SetTextureScaleMode(vram_texture, SDL_SCALEMODE_LINEAR);
            }

            void* pixels;
            int pitch;
            SDL_LockTexture(vram_texture, nullptr, &pixels, &pitch);

            uint32_t* dst = static_cast<uint32_t*>(pixels);
            uint32_t* src = reinterpret_cast<uint32_t*>(gs.vram);

            for (uint32_t y = 0; y < gs.framebuffer1.height; ++y) {
                for (uint32_t x = 0; x < gs.framebuffer1.fbw; ++x) {
                    uint32_t vram_index = y * gs.framebuffer1.fbw + x;
                    dst[vram_index] = src[vram_index];
                }
            }

            SDL_UnlockTexture(vram_texture);

            ImVec2 texture_size = ImVec2(640 * zoom_factor, 480 * zoom_factor); // Always stretch to 640x480
            ImGui::Image(reinterpret_cast<ImTextureID>(vram_texture), texture_size);

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("FRAME_2")) {
            ImGui::Text("Framebuffer 2 Size: %dx%d", gs.framebuffer2.width, gs.framebuffer2.height);

            float tex_w, tex_h;
            if (!vram_texture || SDL_GetTextureSize(vram_texture, &tex_w, &tex_h) != 0 || tex_w != gs.framebuffer2.width || tex_h != gs.framebuffer2.height) {
                if (vram_texture) {
                    SDL_DestroyTexture(vram_texture);
                }
                vram_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_XBGR8888, SDL_TEXTUREACCESS_STREAMING, gs.framebuffer2.width, gs.framebuffer2.height);
                SDL_SetTextureScaleMode(vram_texture, SDL_SCALEMODE_LINEAR);
            }

            void* pixels;
            int pitch;
            SDL_LockTexture(vram_texture, nullptr, &pixels, &pitch);

            uint32_t* dst = static_cast<uint32_t*>(pixels);
            uint32_t* src = reinterpret_cast<uint32_t*>(gs.vram);

            for (uint32_t y = 0; y < gs.framebuffer2.height; ++y) {
                for (uint32_t x = 0; x < gs.framebuffer2.width; ++x) {
                    uint32_t vram_index = y * gs.framebuffer2.width + x;
                    dst[vram_index] = src[vram_index];
                }
            }

            SDL_UnlockTexture(vram_texture);

            ImVec2 texture_size = ImVec2(640 * zoom_factor, 480 * zoom_factor); // Always stretch to 640x480
            ImGui::Image(reinterpret_cast<ImTextureID>(vram_texture), texture_size);

            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    ImGui::End();
}

#include "scheduler.hh"
#include "constants.hh"

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

    static bool show_textures = false;

    static bool show_framebuffer = false;

    argparse::ArgumentParser program("Neo2");

    std::atomic<bool> is_running(false);
    std::atomic<bool> stop_requested(false);  // Flag to stop the thread
    std::mutex status_mutex;  // Mutex for thread-safe status updates
    Breakpoint breakpoints;

    program.add_argument("--bios")
        .help("Path to BIOS")
        .nargs(1);

    program.add_argument("--full-debug")
        .help("Enable all debug windows")
        .nargs(0);

    program.add_argument("--ee-debug")
        .help("Enable EE debug windows")
        .nargs(0);

    program.add_argument("--elf")
        .help("Path to ELF")
        .nargs(1);

    program.add_argument("--ee-breakpoint")
        .help("Breakpoint address for EE")
        .nargs(1);

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

    if (program.is_used("--ee-debug"))
    {
        show_ee_debug = true;
    }

    if (program.is_used("--elf"))
    {
        ee.elf_path = program.get<std::string>("--elf");
        ee.sideload_elf = true;
        ee.set_elf_state(true);
    }

    if (program.is_used("--ee-breakpoint"))
    {
        std::string breakpoints_str = program.get<std::string>("--ee-breakpoint");
        std::stringstream ss(breakpoints_str);
        std::string breakpoint;
        while (std::getline(ss, breakpoint, ','))
        {
            uint32_t address = std::stoul(breakpoint, nullptr, 16);
            breakpoints.add_breakpoint(address, CoreType::EE);
        }
    }

    // Setup SDL
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD))
    {
        printf("Error: SDL_Init(): %s\n", SDL_GetError());
        return;
    }

    // Create window with SDL_Renderer graphics context
    Uint32 window_flags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN | SDL_WINDOW_HIGH_PIXEL_DENSITY;
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
    io.ConfigFlags |= ImGuiConfigFlags_DpiEnableScaleViewports;  // Enable DPI scaling for viewports
    io.ConfigFlags |= ImGuiConfigFlags_DpiEnableScaleFonts;     // Enable DPI scaling for fonts
    io.FontGlobalScale = 1.25f;


    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    // Our state
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    ImGuiDebug debug_interface(*this, this->disassembler);
	std::string bios_file_path;
    std::string status_text = "Idle";
    ImVec4 status_color = ImVec4(1.0f, 0.5f, 0.0f, 1.0f); // Orange for Idle

    this->ee.set_backend(use_jit_ee ? EmulationMode::JIT : EmulationMode::Interpreter);
    this->iop.set_backend(use_jit_iop ? EmulationMode::JIT : EmulationMode::Interpreter);

    Scheduler scheduler;

    // Add tasks to the scheduler
    scheduler.add_task([this](uint64_t cycles) {
        this->ee.execute_cycles(cycles, nullptr);
    }, 1);  // ~4915200/5 cycles

    // Simulate GS vblank once per frame.
    scheduler.add_task([this](uint64_t cycles) {
        this->bus.gs.simul_vblank();
    }, 10000000);  // 4,915,200 cycles*/
    // Simulate GS vblank once per frame.
    scheduler.add_task([this](uint64_t cycles) {
        this->bus.gs.untog_vblank();
    }, 10000105);  // 4,915,200 cycles*/

    /*// Batch draw calls once per frame.
    scheduler.add_task([this](uint64_t cycles) {
        this->bus.gs.batch_draw();
    }, 1000);  // 4,915,200 cycles

    // Execute DMA step very frequently (every 1000 cycles).
    scheduler.add_task([this](uint64_t cycles) {
        this->bus.dmac.dma_step();
    }, 2500);  // every 1000 cycles*/

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
                ImGui::MenuItem("Show Textures", nullptr, &show_textures);
                ImGui::MenuItem("Show Framebuffer", nullptr, &show_framebuffer);
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
            // Add a button to call simul_vblank
    if (ImGui::Button("Simulate VBlank")) {
        this->bus.gs.simul_vblank();
    }
    if (ImGui::Button("Untog VBlank")) {
        this->bus.gs.untog_vblank();
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

        if (show_textures) {
            render_textures(renderer, bus.gs);
        }

        if (show_framebuffer) {
            render_framebuffer(renderer, bus.gs);
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
                    std::thread ee_thread([this, &is_running, &breakpoints, &stop_requested, &status_text, &status_color, &scheduler]() {
                        while (!stop_requested) {
                            // Tick the scheduler
                            scheduler.tick(); // Tick the scheduler with 1000 cycles

                            // Call this before the step
                            /*if (client_fd == -1)
                            {
                                memset(&ee.cop0_registers, 0, sizeof(ee.cop0_registers));
                                ee.cop0_registers[12] = 0x70400004;
                                ee.cop0_registers[15] = 0x2E20;
                                ee.cop0_registers[16] = 0x440;
                                connectToPCSX2();
                            }

                            if (!compareRegistersWithPCSX2(&ee))*/
                            //this->ee.run(&breakpoints);
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
