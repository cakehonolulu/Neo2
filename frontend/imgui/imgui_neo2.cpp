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
#include "imgui_impl_sdlgpu3.h"
#include "imgui_impl_sdlrenderer3.h"
#include "imgui_impl_opengl3.h"
#include "log/log_imgui.hh"
#include <SDL3/SDL.h>
#include <SDL3/SDL_surface.h>
#include <stdio.h>

#include "ImGuiFileDialog.h"
#include <argparse/argparse.hpp>

#include "scheduler.hh"
#include "constants.hh"

SDL_GPUGraphicsPipeline *hw_pipeline = nullptr;
SDL_GPUBuffer *hw_vertex_buffer = nullptr;

SDL_GPUShader *LoadCompiledShader(SDL_GPUDevice *gpu_device, const char *filePath, SDL_GPUShaderStage stage)
{
    // Load the compiled shader file into memory.
    size_t codeSize = 0;
    void *code = SDL_LoadFile(filePath, &codeSize);
    if (!code)
    {
        SDL_Log("LoadCompiledShader: Failed to load shader file '%s': %s", filePath, SDL_GetError());
        return nullptr;
    }

    SDL_GPUShaderFormat backendFormats = SDL_GetGPUShaderFormats(gpu_device);

    // Fill in the shader create info.
    SDL_GPUShaderCreateInfo shaderInfo;
    SDL_zero(shaderInfo);
    shaderInfo.code = reinterpret_cast<const Uint8 *>(code);
    shaderInfo.code_size = codeSize;
    shaderInfo.entrypoint = "main"; // Entry point function name
    // For the D3D12 backend, we use DXIL.
    shaderInfo.stage = stage;
    // Set these counts as needed by your shaders.
    shaderInfo.num_samplers = 0;
    shaderInfo.num_uniform_buffers = 0;

    if (backendFormats & SDL_GPU_SHADERFORMAT_SPIRV)
    {
        printf("SPIRV Shader\n");
        shaderInfo.format = SDL_GPU_SHADERFORMAT_SPIRV;
    }
    else if (backendFormats & SDL_GPU_SHADERFORMAT_DXBC)
    {
        printf("DXBC Shader\n");
        shaderInfo.format = SDL_GPU_SHADERFORMAT_DXBC;
    }
    else if (backendFormats & SDL_GPU_SHADERFORMAT_DXIL)
    {
        printf("DXIL Shader\n");
        shaderInfo.format = SDL_GPU_SHADERFORMAT_DXIL;
    }
    else
    {
        SDL_Log("%s", "Unrecognized backend shader format!");
        return NULL;
    }

    // Create the shader.
    SDL_GPUShader *shader = SDL_CreateGPUShader(gpu_device, &shaderInfo);
    if (!shader)
    {
        SDL_Log("LoadCompiledShader: Failed to create shader from file '%s': %s", filePath, SDL_GetError());
    }

    // Free the loaded shader code since SDL_CreateGPUShader copies it.
    SDL_free(code);
    return shader;
}

SDL_GPUShader *CompileShader(SDL_GPUDevice *gpu_device, const char *source, SDL_GPUShaderStage stage)
{
    // Fill in the shader create info.
    SDL_GPUShaderCreateInfo shaderInfo;
    SDL_zero(shaderInfo);
    shaderInfo.code_size = strlen(source) + 1; // including null terminator
    shaderInfo.code = reinterpret_cast<const Uint8 *>(source);
    // For simplicity, use "main" as the entry point.
    shaderInfo.entrypoint = "main";
    // Assume for this example that our device accepts our GLSL as SPIR-V (in a real app, precompile to SPIR-V or DXIL).
    shaderInfo.format = SDL_GPU_SHADERFORMAT_DXIL;
    shaderInfo.stage = stage;
    shaderInfo.num_samplers = 1;        // Adjust if needed.
    shaderInfo.num_uniform_buffers = 1; // For example, for a projection matrix.

    SDL_GPUShader *shader = SDL_CreateGPUShader(gpu_device, &shaderInfo);
    if (!shader)
    {
        SDL_Log("Failed to create shader: %s", SDL_GetError());
    }
    return shader;
}

// --- Setup the graphics pipeline ---
SDL_GPUGraphicsPipeline *CreatePipeline(SDL_GPUDevice *gpu_device, SDL_Window *window, SDL_GPUShader *vertexShader,
                                        SDL_GPUShader *fragmentShader)
{
    SDL_GPUGraphicsPipelineCreateInfo pipelineInfo;
    SDL_zero(pipelineInfo);

    // Setup the color target info.
    pipelineInfo.target_info.num_color_targets = 1;
    SDL_GPUColorTargetDescription colorTargetDesc;
    SDL_zero(colorTargetDesc);
    colorTargetDesc.format = SDL_GetGPUSwapchainTextureFormat(gpu_device, window);
    pipelineInfo.target_info.color_target_descriptions = &colorTargetDesc;

    // Setup the vertex input state.
    SDL_GPUVertexBufferDescription vbDesc;
    SDL_zero(vbDesc);
    vbDesc.slot = 0;
    vbDesc.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    vbDesc.pitch = sizeof(float) * 3 + sizeof(Uint32); // 3 floats (position) + 1 uint (color)

    // Two vertex attributes: position and color.
    SDL_GPUVertexAttribute attributes[2];
    SDL_zero(attributes[0]);
    attributes[0].location = 0;
    attributes[0].buffer_slot = 0;
    attributes[0].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    attributes[0].offset = 0;

    SDL_zero(attributes[1]);
    attributes[1].location = 1;
    attributes[1].buffer_slot = 0;
    // Use UINT for the raw 32-bit color.
    attributes[1].format = SDL_GPU_VERTEXELEMENTFORMAT_UINT;
    attributes[1].offset = sizeof(float) * 3;

    SDL_GPUVertexInputState vertexInputState;
    SDL_zero(vertexInputState);
    vertexInputState.vertex_buffer_descriptions = &vbDesc;
    vertexInputState.num_vertex_buffers = 1;
    vertexInputState.vertex_attributes = attributes;
    vertexInputState.num_vertex_attributes = 2;

    pipelineInfo.vertex_input_state = vertexInputState;
    pipelineInfo.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;

    // Set the shaders.
    pipelineInfo.vertex_shader = vertexShader;
    pipelineInfo.fragment_shader = fragmentShader;

    printf("CreatePipeline: Created all aux. structs\n");
    // Create the pipeline.
    SDL_GPUGraphicsPipeline *pipeline = SDL_CreateGPUGraphicsPipeline(gpu_device, &pipelineInfo);
    if (!pipeline)
    {
        SDL_Log("CreatePipeline: Failed to create graphics pipeline: %s", SDL_GetError());
    }
    return pipeline;
}

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

void ImGui_Neo2::run(int argc, char **argv)
{
    Disassembler disassembler;
    SDL_GPUTexture *swapchain_texture, *emu_texure = nullptr;
    SDL_Surface *vram = nullptr;
    SDL_GPUCommandBuffer *command_buffer = nullptr;

    SDL_GPUBuffer *quad_vertex_buffer = nullptr;

    float zoom_factor = 1.0f;

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

    bus.load_bios("C:\\Users\\joel.bueno\\source\\repos\\Neo2\\scph10000.bin");
    ee.elf_path = "C:\\Users\\joel.bueno\\source\\repos\\Neo2\\3stars.elf";
    ee.sideload_elf = true;
    ee.set_elf_state(true);

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

    SDL_Window *window =
        SDL_CreateWindow("Neo2 - ImGui + SDL3", 1280, 720, SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (window == nullptr)
    {
        printf("Error: SDL_CreateWindow(): %s\n", SDL_GetError());
        return;
    }
    SDL_GPUDevice *gpu_device = SDL_CreateGPUDevice(
        SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_DXIL | SDL_GPU_SHADERFORMAT_METALLIB, true, nullptr);
    if (gpu_device == nullptr)
    {
        printf("Error: SDL_CreateGPUDevice(): %s\n", SDL_GetError());
        return;
    }

    SDL_GPUShaderFormat backendFormats = SDL_GetGPUShaderFormats(gpu_device);
    this->bus.gs.set_render_mode((RenderMode)backendFormats);

    // Claim window for GPU Device
    if (!SDL_ClaimWindowForGPUDevice(gpu_device, window))
    {
        printf("Error: SDL_ClaimWindowForGPUDevice(): %s\n", SDL_GetError());
        return;
    }
    SDL_SetGPUSwapchainParameters(gpu_device, window, SDL_GPU_SWAPCHAINCOMPOSITION_SDR, SDL_GPU_PRESENTMODE_MAILBOX);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.FontGlobalScale = 1.25f;

    // Hazy Dark style by kaitabuchi314 from ImThemes
	ImGuiStyle& style = ImGui::GetStyle();
	
	style.Alpha = 1.0f;
	style.DisabledAlpha = 0.6000000238418579f;
	style.WindowPadding = ImVec2(5.5f, 8.300000190734863f);
	style.WindowRounding = 4.5f;
	style.WindowBorderSize = 1.0f;
	style.WindowMinSize = ImVec2(32.0f, 32.0f);
	style.WindowTitleAlign = ImVec2(0.0f, 0.5f);
	style.WindowMenuButtonPosition = ImGuiDir_Left;
	style.ChildRounding = 3.200000047683716f;
	style.ChildBorderSize = 1.0f;
	style.PopupRounding = 2.700000047683716f;
	style.PopupBorderSize = 1.0f;
	style.FramePadding = ImVec2(4.0f, 3.0f);
	style.FrameRounding = 2.400000095367432f;
	style.FrameBorderSize = 0.0f;
	style.ItemSpacing = ImVec2(8.0f, 4.0f);
	style.ItemInnerSpacing = ImVec2(4.0f, 4.0f);
	style.CellPadding = ImVec2(4.0f, 2.0f);
	style.IndentSpacing = 21.0f;
	style.ColumnsMinSpacing = 6.0f;
	style.ScrollbarSize = 14.0f;
	style.ScrollbarRounding = 9.0f;
	style.GrabMinSize = 10.0f;
	style.GrabRounding = 3.200000047683716f;
	style.TabRounding = 3.5f;
	style.TabBorderSize = 1.0f;
	style.TabMinWidthForCloseButton = 0.0f;
	style.ColorButtonPosition = ImGuiDir_Right;
	style.ButtonTextAlign = ImVec2(0.5f, 0.5f);
	style.SelectableTextAlign = ImVec2(0.0f, 0.0f);
	
	style.Colors[ImGuiCol_Text] = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
	style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.4980392158031464f, 0.4980392158031464f, 0.4980392158031464f, 1.0f);
	style.Colors[ImGuiCol_WindowBg] = ImVec4(0.05882352963089943f, 0.05882352963089943f, 0.05882352963089943f, 0.9399999976158142f);
	style.Colors[ImGuiCol_ChildBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
	style.Colors[ImGuiCol_PopupBg] = ImVec4(0.0784313753247261f, 0.0784313753247261f, 0.0784313753247261f, 0.9399999976158142f);
	style.Colors[ImGuiCol_Border] = ImVec4(0.4274509847164154f, 0.4274509847164154f, 0.4980392158031464f, 0.5f);
	style.Colors[ImGuiCol_BorderShadow] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
	style.Colors[ImGuiCol_FrameBg] = ImVec4(0.1372549086809158f, 0.1725490242242813f, 0.2274509817361832f, 0.5400000214576721f);
	style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.2117647081613541f, 0.2549019753932953f, 0.3019607961177826f, 0.4000000059604645f);
	style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.04313725605607033f, 0.0470588244497776f, 0.0470588244497776f, 0.6700000166893005f);
	style.Colors[ImGuiCol_TitleBg] = ImVec4(0.03921568766236305f, 0.03921568766236305f, 0.03921568766236305f, 1.0f);
	style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.0784313753247261f, 0.08235294371843338f, 0.09019608050584793f, 1.0f);
	style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.0f, 0.0f, 0.0f, 0.5099999904632568f);
	style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.1372549086809158f, 0.1372549086809158f, 0.1372549086809158f, 1.0f);
	style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.01960784383118153f, 0.01960784383118153f, 0.01960784383118153f, 0.5299999713897705f);
	style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.3098039329051971f, 0.3098039329051971f, 0.3098039329051971f, 1.0f);
	style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.407843142747879f, 0.407843142747879f, 0.407843142747879f, 1.0f);
	style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.5098039507865906f, 0.5098039507865906f, 0.5098039507865906f, 1.0f);
	style.Colors[ImGuiCol_CheckMark] = ImVec4(0.7176470756530762f, 0.7843137383460999f, 0.843137264251709f, 1.0f);
	style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.47843137383461f, 0.5254902243614197f, 0.572549045085907f, 1.0f);
	style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.2901960909366608f, 0.3176470696926117f, 0.3529411852359772f, 1.0f);
	style.Colors[ImGuiCol_Button] = ImVec4(0.1490196138620377f, 0.1607843190431595f, 0.1764705926179886f, 0.4000000059604645f);
	style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.1372549086809158f, 0.1450980454683304f, 0.1568627506494522f, 1.0f);
	style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.0784313753247261f, 0.08627451211214066f, 0.09019608050584793f, 1.0f);
	style.Colors[ImGuiCol_Header] = ImVec4(0.196078434586525f, 0.2156862765550613f, 0.239215686917305f, 0.3100000023841858f);
	style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.1647058874368668f, 0.1764705926179886f, 0.1921568661928177f, 0.800000011920929f);
	style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.07450980693101883f, 0.08235294371843338f, 0.09019608050584793f, 1.0f);
	style.Colors[ImGuiCol_Separator] = ImVec4(0.4274509847164154f, 0.4274509847164154f, 0.4980392158031464f, 0.5f);
	style.Colors[ImGuiCol_SeparatorHovered] = ImVec4(0.239215686917305f, 0.3254902064800262f, 0.4235294163227081f, 0.7799999713897705f);
	style.Colors[ImGuiCol_SeparatorActive] = ImVec4(0.2745098173618317f, 0.3803921639919281f, 0.4980392158031464f, 1.0f);
	style.Colors[ImGuiCol_ResizeGrip] = ImVec4(0.2901960909366608f, 0.3294117748737335f, 0.3764705955982208f, 0.2000000029802322f);
	style.Colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.239215686917305f, 0.2980392277240753f, 0.3686274588108063f, 0.6700000166893005f);
	style.Colors[ImGuiCol_ResizeGripActive] = ImVec4(0.1647058874368668f, 0.1764705926179886f, 0.1882352977991104f, 0.949999988079071f);
	style.Colors[ImGuiCol_Tab] = ImVec4(0.1176470592617989f, 0.125490203499794f, 0.1333333402872086f, 0.8619999885559082f);
	style.Colors[ImGuiCol_TabHovered] = ImVec4(0.3294117748737335f, 0.407843142747879f, 0.501960813999176f, 0.800000011920929f);
	style.Colors[ImGuiCol_TabActive] = ImVec4(0.2431372553110123f, 0.2470588237047195f, 0.2549019753932953f, 1.0f);
	style.Colors[ImGuiCol_TabUnfocused] = ImVec4(0.06666667014360428f, 0.1019607856869698f, 0.1450980454683304f, 0.9724000096321106f);
	style.Colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.1333333402872086f, 0.2588235437870026f, 0.4235294163227081f, 1.0f);
	style.Colors[ImGuiCol_PlotLines] = ImVec4(0.6078431606292725f, 0.6078431606292725f, 0.6078431606292725f, 1.0f);
	style.Colors[ImGuiCol_PlotLinesHovered] = ImVec4(1.0f, 0.4274509847164154f, 0.3490196168422699f, 1.0f);
	style.Colors[ImGuiCol_PlotHistogram] = ImVec4(0.8980392217636108f, 0.6980392336845398f, 0.0f, 1.0f);
	style.Colors[ImGuiCol_PlotHistogramHovered] = ImVec4(1.0f, 0.6000000238418579f, 0.0f, 1.0f);
	style.Colors[ImGuiCol_TableHeaderBg] = ImVec4(0.1882352977991104f, 0.1882352977991104f, 0.2000000029802322f, 1.0f);
	style.Colors[ImGuiCol_TableBorderStrong] = ImVec4(0.3098039329051971f, 0.3098039329051971f, 0.3490196168422699f, 1.0f);
	style.Colors[ImGuiCol_TableBorderLight] = ImVec4(0.2274509817361832f, 0.2274509817361832f, 0.2470588237047195f, 1.0f);
	style.Colors[ImGuiCol_TableRowBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
	style.Colors[ImGuiCol_TableRowBgAlt] = ImVec4(1.0f, 1.0f, 1.0f, 0.05999999865889549f);
	style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.2588235437870026f, 0.5882353186607361f, 0.9764705896377563f, 0.3499999940395355f);
	style.Colors[ImGuiCol_DragDropTarget] = ImVec4(1.0f, 1.0f, 0.0f, 0.8999999761581421f);
	style.Colors[ImGuiCol_NavHighlight] = ImVec4(0.2588235437870026f, 0.5882353186607361f, 0.9764705896377563f, 1.0f);
	style.Colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.0f, 1.0f, 1.0f, 0.699999988079071f);
	style.Colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.800000011920929f, 0.800000011920929f, 0.800000011920929f, 0.2000000029802322f);
	style.Colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.800000011920929f, 0.800000011920929f, 0.800000011920929f, 0.3499999940395355f);

    // Setup Platform/Renderer backends
    ImGui_ImplSDL3_InitForSDLGPU(window);
    ImGui_ImplSDLGPU3_InitInfo init_info = {};
    init_info.Device = gpu_device;
    init_info.ColorTargetFormat = SDL_GetGPUSwapchainTextureFormat(gpu_device, window);
    init_info.MSAASamples = SDL_GPU_SAMPLECOUNT_1;
    ImGui_ImplSDLGPU3_Init(&init_info);

    // Our state
    ImVec4 clear_color = ImVec4(0.14f, 0.14f, 0.14f, 1.00f);

    ImGuiDebug debug_interface(*this, this->disassembler);
	std::string bios_file_path;
    std::string status_text = "Idle";
    ImVec4 status_color = ImVec4(1.0f, 0.5f, 0.0f, 1.0f); // Orange for Idle

    this->ee.set_backend(use_jit_ee ? EmulationMode::JIT : EmulationMode::Interpreter);
    this->iop.set_backend(use_jit_iop ? EmulationMode::JIT : EmulationMode::Interpreter);

    Scheduler scheduler;

    bool draw = false;

    int gs_vblank_event_id = scheduler.register_function([this](uint64_t cycles) {
        this->bus.gs.simul_vblank();
    });

    int gs_vblank_task_id = scheduler.register_function([this, &scheduler, &gs_vblank_event_id](uint64_t cycles) {
        scheduler.add_event(gs_vblank_event_id, GS_VBLANK_DELAY, "VBlank Delay");
    });

    int vblank_end_id = scheduler.register_function([this, &draw](uint64_t cycles) {
        this->bus.gs.untog_vblank();
        draw = true;
        frame_ended = true;
    });

    SDL_GPUSamplerCreateInfo samplerCreateInfo = {
        .min_filter = SDL_GPU_FILTER_NEAREST,
        .mag_filter = SDL_GPU_FILTER_NEAREST,
        .mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST,
        .address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
        .address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
        .address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
    };

    static SDL_GPUSampler *sampler = SDL_CreateGPUSampler(gpu_device, &samplerCreateInfo);

    // Compile shaders (using your helper function or your own method)
    SDL_GPUShader *vertexShader = LoadCompiledShader(gpu_device, "vertexShader.spv", SDL_GPU_SHADERSTAGE_VERTEX);
    SDL_GPUShader *fragmentShader = LoadCompiledShader(gpu_device, "fragmentShader.spv", SDL_GPU_SHADERSTAGE_FRAGMENT);

    // Create the graphics pipeline using the compiled shaders.
    hw_pipeline = CreatePipeline(gpu_device, window, vertexShader, fragmentShader);

    // Optionally, release shaders if your pipeline takes ownership.
    SDL_ReleaseGPUShader(gpu_device, vertexShader);
    SDL_ReleaseGPUShader(gpu_device, fragmentShader);

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
        ImGui_ImplSDLGPU3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        int fbWidth = this->bus.gs.framebuffer1.width;
        int fbHeight = this->bus.gs.framebuffer1.height;

        if (fbWidth != prev_fb_width || fbHeight != prev_fb_height)
        {
            prev_fb_width = this->bus.gs.framebuffer1.width;
            prev_fb_height = this->bus.gs.framebuffer1.height;

            printf("Framebuffer size changed, updating texture (New: %dx%d)...\n", this->bus.gs.framebuffer1.width,
                       this->bus.gs.framebuffer1.height);
            SDL_GPUTextureCreateInfo textureCreateInfo{
                .format = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
                .usage = SDL_GPU_TEXTUREUSAGE_SAMPLER | SDL_GPU_TEXTUREUSAGE_COLOR_TARGET,
                .width = static_cast<Uint32>(639),
                .height = static_cast<Uint32>(224),
                .layer_count_or_depth = 1,
                .num_levels = 1,
            };
            emu_texure = SDL_CreateGPUTexture(gpu_device, &textureCreateInfo);
            if (!emu_texure)
            {
                printf("Couldn't create texture from VRAM!\n");
            }
            else
            {
                printf("New texture created successfully!\n");
            }

            SDL_SetGPUTextureName(gpu_device, emu_texure, "vram");
        }

        // Use fixed native dimensions for display
        const int nativeWidth = 640;
        const int nativeHeight = 480;

        // Get window size from ImGui IO
        float windowWidth = io.DisplaySize.x;
        float windowHeight = io.DisplaySize.y;

        // Assume your menubar and status bar each use the height of one frame
        float menubarHeight = ImGui::GetFrameHeight();
        float statusbarHeight = ImGui::GetFrameHeight();

        // Compute the available area for the background (between menubar and status bar)
        float availableWidth = windowWidth;
        float availableHeight = windowHeight - menubarHeight - statusbarHeight;

        // Calculate the maximum integer scale factor that fits in the available area
        int scaleFactor =
            std::max(1, (int)std::min(availableWidth / (float)nativeWidth, availableHeight / (float)nativeHeight));

        // Compute the scaled dimensions based on the native 640x480 resolution
        int scaledWidth = nativeWidth * scaleFactor;
        int scaledHeight = nativeHeight * scaleFactor;

        // Center the scaled image in the available area
        float posX = (availableWidth - scaledWidth) * 0.5f;
        float posY = menubarHeight + (availableHeight - scaledHeight) * 0.5f;

        if (emu_texure != nullptr)
        {
            ImDrawList *bgDrawList = ImGui::GetBackgroundDrawList();
            /* bgDrawList->AddImage((ImTextureID)(emu_texure), ImVec2(posX, posY),
                                 ImVec2(posX + scaledWidth, posY + scaledHeight));*/

            SDL_GPUTextureSamplerBinding texture_sampler_binding[2];
            texture_sampler_binding[0].texture = emu_texure;
            texture_sampler_binding[0].sampler = sampler;
            //ImGui::Image(ImTextureID(texture_sampler_binding), ImVec2(720, 480));
        }

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
            //render_textures(bus.gs, zoom_factor);
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
        //render_primitive_viewer(bus.gs);

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
                    std::thread ee_thread([this, &is_running, &breakpoints, &stop_requested, &status_text,
                                           &status_color, &scheduler, &gs_vblank_task_id, &vblank_end_id,
                                           &gpu_device, &emu_texure, &command_buffer]() {
                        while (!stop_requested) {
                            frame_ended = false;
                            scheduler.add_event(gs_vblank_task_id, VBLANK_START_CYCLES, "VBlank Start");
                            scheduler.add_event(vblank_end_id, PS2_CYCLES_PER_FRAME, "VBlank End");
                            
                            while (!frame_ended && !stop_requested) {
                                // Calculate target cycles to run for this iteration
                                unsigned int target_cycles = scheduler.calculate_run_cycles();

                                // Record the starting cycle count (assumes core->cycles is updated by EE execution)
                                uint64_t start_cycles = this->ee.cycles;
                                
                                // Execute the target number of cycles (this->ee.execute_cycles() will update core->cycles)
                                if (!stop_requested) {
                                    this->ee.execute_cycles(target_cycles, &breakpoints);
                                }
                                
                                // Determine the actual number of cycles executed
                                uint64_t executed_cycles = this->ee.cycles - start_cycles;
                                
                                // Update scheduler cycle counts with the real executed cycle count
                                if (!stop_requested) {
                                    scheduler.update_cycle_counts(executed_cycles);
                                }
                                
                                // Process any pending scheduler events
                                if (!stop_requested) {
                                    scheduler.process_events();
                                }
                            }
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

            ImGui::SameLine();
            const char *render_modes[] = {"Software",    "Vulkan",      "DX12 (DXBC)", "DX12 (DXIL)",
                                          "Metal (MSL)", "Metal (LIB)", "OpenGL"};
            RenderMode current_render_mode = this->bus.gs.render_mode;

            int current_render_mode_index = 0;
            switch (current_render_mode)
            {
            case RenderMode::Software:
                current_render_mode_index = 0;
                break;
            case RenderMode::Vulkan:
                current_render_mode_index = 1;
                break;
            case RenderMode::DX12_DXBC:
                current_render_mode_index = 2;
                break;
            case RenderMode::DX12_DXIL:
                current_render_mode_index = 3;
                break;
            case RenderMode::Metal_MSL:
                current_render_mode_index = 4;
                break;
            case RenderMode::Metal_LIB:
                current_render_mode_index = 5;
                break;
            case RenderMode::OpenGL:
                current_render_mode_index = 6;
                break;
            default:
                break;
            }

            ImGui::SetNextItemWidth(150); // Set the width of the combobox
            if (ImGui::Combo("Render Mode", &current_render_mode_index, render_modes, IM_ARRAYSIZE(render_modes)))
            {
                switch (current_render_mode_index)
                {
                case 0:
                    this->bus.gs.set_render_mode(RenderMode::Software);
                    break;
                case 1:
                    this->bus.gs.set_render_mode(RenderMode::Vulkan);
                    break;
                case 2:
                    this->bus.gs.set_render_mode(RenderMode::DX12_DXBC);
                    break;
                case 3:
                    this->bus.gs.set_render_mode(RenderMode::DX12_DXIL);
                    break;
                case 4:
                    this->bus.gs.set_render_mode(RenderMode::Metal_MSL);
                    break;
                case 5:
                    this->bus.gs.set_render_mode(RenderMode::Metal_LIB);
                    break;
                case 6:
                    this->bus.gs.set_render_mode(RenderMode::OpenGL);
                    break;
                default:
                    break;
                }
            }
            ImGui::SameLine();
            text_height = ImGui::GetTextLineHeight();
            button_height = ImGui::GetFrameHeight();
            vertical_offset = (text_height - button_height) / 2.0f;
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + vertical_offset);

            {
                // Get the full width of the status bar window.
                float windowWidth = ImGui::GetWindowWidth();
                // Calculate the size of the text you want to render.
                ImVec2 fbTextSize = ImGui::CalcTextSize("Framebuffer: 0000x0000");
                // Set a margin from the right edge (in pixels)
                float margin = 5.0f;
                // Set the cursor position X so that the text starts at (window width - text width - margin)
                ImGui::SetCursorPosX(windowWidth - fbTextSize.x - margin);
                // Render the framebuffer dimensions.
                ImGui::Text("Framebuffer: %dx%d", this->bus.gs.framebuffer1.width, this->bus.gs.framebuffer1.height);
            }
            ImGui::PopStyleColor(3);
            ImGui::End();
        }
        
        ImGui::Render();
        
        ImDrawData *draw_data = ImGui::GetDrawData();
        const bool is_minimized = (draw_data->DisplaySize.x <= 0.0f || draw_data->DisplaySize.y <= 0.0f);

        command_buffer = SDL_AcquireGPUCommandBuffer(gpu_device); // Acquire a GPU command buffer

        SDL_AcquireGPUSwapchainTexture(command_buffer, window, &swapchain_texture, nullptr,
                                       nullptr); // Acquire a swapchain texture

        if (swapchain_texture != nullptr)
        {
            SDL_GPUBlitInfo blitInfo;
            SDL_zero(blitInfo);

            blitInfo.source.texture = emu_texure;
            blitInfo.source.mip_level = 0;
            blitInfo.source.layer_or_depth_plane = 0;
            blitInfo.source.x = 0;
            blitInfo.source.y = 0;
            // Use the active framebuffer width/height here.
            blitInfo.source.w = static_cast<Uint32>(this->bus.gs.framebuffer1.width);
            blitInfo.source.h = static_cast<Uint32>(this->bus.gs.framebuffer1.height);

            // Next, set up the destination region using your computed values.
            blitInfo.destination.texture = swapchain_texture; // The swapchain texture acquired earlier.
            blitInfo.destination.mip_level = 0;
            blitInfo.destination.layer_or_depth_plane = 0;
            blitInfo.destination.x = static_cast<Uint32>(posX);
            blitInfo.destination.y = static_cast<Uint32>(posY);
            blitInfo.destination.w = static_cast<Uint32>(scaledWidth);
            blitInfo.destination.h = static_cast<Uint32>(scaledHeight);

            // Choose a load operation. If you want to preserve previous contents (for composition), use LOAD;
            // if you want to clear before drawing, use CLEAR and specify clear_color.
            blitInfo.load_op = SDL_GPU_LOADOP_CLEAR;
            blitInfo.clear_color = SDL_FColor{clear_color.x, clear_color.y, clear_color.z, clear_color.w};

            // Set flip mode and filter mode as desired.
            blitInfo.flip_mode = SDL_FLIP_NONE;
            blitInfo.filter = SDL_GPU_FILTER_NEAREST;
            blitInfo.cycle = false;

            // Now, blit the texture using the command buffer.
            // Note: SDL_BlitGPUTexture must be called outside any render pass.
            SDL_BlitGPUTexture(command_buffer, &blitInfo);
        }
        if (swapchain_texture != nullptr && !is_minimized)
        {
            // This is mandatory: call Imgui_ImplSDLGPU3_PrepareDrawData() to upload the vertex/index buffer!
            Imgui_ImplSDLGPU3_PrepareDrawData(draw_data, command_buffer);

            // Setup and start a render pass
            SDL_GPUColorTargetInfo target_info = {};
            target_info.texture = swapchain_texture;
            target_info.clear_color = SDL_FColor{clear_color.x, clear_color.y, clear_color.z, clear_color.w};
            //target_info.load_op = SDL_GPU_LOADOP_CLEAR;
            target_info.store_op = SDL_GPU_STOREOP_STORE;
            target_info.mip_level = 0;
            target_info.layer_or_depth_plane = 0;
            target_info.cycle = false;
            SDL_GPURenderPass *render_pass = SDL_BeginGPURenderPass(command_buffer, &target_info, 1, nullptr);

            // Render ImGui
            ImGui_ImplSDLGPU3_RenderDrawData(draw_data, command_buffer, render_pass);

            SDL_EndGPURenderPass(render_pass);
        }
        
        if (draw == true)
        {
                if (!emu_texure)
                {
                    Logger::error("Couldn't create texture from VRAM!");
                }
                else
                {
                    Uint32 transferSize = this->bus.gs.framebuffer1.fbw * this->bus.gs.framebuffer1.height * 4;
                    SDL_GPUTransferBufferCreateInfo tbci;
                    SDL_zero(tbci);
                    tbci.size = transferSize;
                    tbci.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;

                    SDL_GPUTransferBuffer *tbuf = SDL_CreateGPUTransferBuffer(gpu_device, &tbci);
                    if (!tbuf)
                    {
                        printf("Failed to create transfer buffer: %s\n", SDL_GetError());
                        // Handle error appropriately.
                        return;
                    }

                    // Acquire a GPU command buffer and begin a copy pass.
                    SDL_GPUCopyPass *copy_pass = SDL_BeginGPUCopyPass(command_buffer);
                    if (!copy_pass)
                    {
                        printf("Failed to begin GPU copy pass: %s\n", SDL_GetError());
                        return;
                    }

                    // Map the transfer buffer and copy the pixel data into it.
                    void *textureTransferPtr = SDL_MapGPUTransferBuffer(gpu_device, tbuf, true);
                    if (!textureTransferPtr)
                    {
                        printf("Failed to map transfer buffer: %s\n", SDL_GetError());
                        // Handle error...
                    }
                    
                    SDL_memcpy(textureTransferPtr, this->bus.gs.vram, transferSize);

                    // Prepare the source info structure.
                    SDL_GPUTextureTransferInfo srcInfo;
                    SDL_zero(srcInfo);
                    srcInfo.transfer_buffer = tbuf;
                    srcInfo.offset = 0;
                    srcInfo.pixels_per_row = static_cast<Uint32>(this->bus.gs.framebuffer1.fbw);
                    srcInfo.rows_per_layer = static_cast<Uint32>(this->bus.gs.framebuffer1.height); // Number of rows

                    // Prepare the destination region structure.
                    SDL_GPUTextureRegion destRegion;
                    SDL_zero(destRegion);
                    destRegion.texture = emu_texure;
                    destRegion.mip_level = 0;
                    destRegion.layer = 0;
                    destRegion.x = 0;
                    destRegion.y = 0;
                    destRegion.z = 0;
                    destRegion.w = static_cast<Uint32>(this->bus.gs.framebuffer1.width);
                    destRegion.h = static_cast<Uint32>(this->bus.gs.framebuffer1.height);
                    destRegion.d = 1;

                    // Upload the data from the transfer buffer to the GPU texture.
                    SDL_UploadToGPUTexture(copy_pass, &srcInfo, &destRegion, false);

                    SDL_UnmapGPUTransferBuffer(gpu_device, tbuf);
                    // End the copy pass and submit the command buffer.
                    SDL_EndGPUCopyPass(copy_pass);

                    SDL_ReleaseGPUTransferBuffer(gpu_device, tbuf);
                }
            
                draw = false;
        }
        
        // Submit the command buffer
        SDL_SubmitGPUCommandBuffer(command_buffer);
    }
#ifdef __EMSCRIPTEN__
    EMSCRIPTEN_MAINLOOP_END;
#endif

    // Cleanup
    SDL_WaitForGPUIdle(gpu_device);
    ImGui_ImplSDL3_Shutdown();
    ImGui_ImplSDLGPU3_Shutdown();
    ImGui::DestroyContext();

    SDL_ReleaseWindowFromGPUDevice(gpu_device, window);
    SDL_DestroyGPUDevice(gpu_device);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
