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
#include "imgui_impl_opengl3.h"
#include "log/log_imgui.hh"
#include <SDL3/SDL.h>
#include <GL/glx.h>
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

#include "scheduler.hh"
#include "constants.hh"

struct MyRect {
    ImVec2 Min;
    ImVec2 Max;
};

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

unsigned int create_texture_from_vram(GS& gs, const GS::Texture& texture) {
    unsigned int texture_id;
    glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glPixelStorei(GL_UNPACK_ROW_LENGTH, gs.framebuffer1.fbw);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, static_cast<int>(texture.width), static_cast<int>(texture.height), 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture.width, texture.height, GL_RGBA, GL_UNSIGNED_BYTE, gs.vram + texture.address);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

    glBindTexture(GL_TEXTURE_2D, 0);

    return texture_id;
}

void render_textures(GS& gs, float zoom_factor) {
    ImGui::Begin("Textures", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    const auto& textures = gs.get_textures();
    static int selected_texture_index = -1;
    static std::vector<bool> texture_windows_open(textures.size(), false);
    static std::vector<unsigned int> opengl_textures(textures.size(), 0);

    ImGui::Text("Available Textures:");
    ImGui::Separator();

    for (size_t i = 0; i < textures.size(); ++i) {
        const auto& texture = textures[i];
        if (ImGui::Selectable(texture.name.c_str(), selected_texture_index == static_cast<int>(i))) {
            selected_texture_index = static_cast<int>(i);
            texture_windows_open[i] = true;
            if (opengl_textures[i] == 0) {
                opengl_textures[i] = create_texture_from_vram(gs, texture);
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
            ImGui::Image((ImTextureID)(opengl_textures[i]), texture_size);

            ImGui::End();

            if (!open) {
                //glDeleteTextures(1, &opengl_textures[i]);
                //opengl_textures[i] = 0;
            }
        }
    }

    ImGui::End();
}

// Compute a bounding rectangle for a set of vertices.
MyRect compute_bounding_rect(const std::vector<Vertex>& vertices) {
    float min_x = FLT_MAX, min_y = FLT_MAX;
    float max_x = -FLT_MAX, max_y = -FLT_MAX;
    for (const Vertex& v : vertices) {
        if (v.x < min_x) min_x = v.x;
        if (v.y < min_y) min_y = v.y;
        if (v.x > max_x) max_x = v.x;
        if (v.y > max_y) max_y = v.y;
    }
    return MyRect{ImVec2(min_x, min_y), ImVec2(max_x, max_y)};
}

// Helper to compute an average color from the vertices (assumes color stored as 0xRRGGBBAA).
ImVec4 compute_average_color(const std::vector<Vertex>& vertices) {
    uint32_t sumR = 0, sumG = 0, sumB = 0, sumA = 0;
    for (const Vertex& v : vertices) {
        uint8_t r = (v.color >> 24) & 0xFF;
        uint8_t g = (v.color >> 16) & 0xFF;
        uint8_t b = (v.color >> 8) & 0xFF;
        uint8_t a = v.color & 0xFF;
        sumR += r;
        sumG += g;
        sumB += b;
        sumA += a;
    }
    size_t count = vertices.size();
    return ImVec4(
        static_cast<float>(sumR) / (count * 255.0f),
        static_cast<float>(sumG) / (count * 255.0f),
        static_cast<float>(sumB) / (count * 255.0f),
        static_cast<float>(sumA) / (count * 255.0f)
    );
}

// Render the primitive viewer window, including a preview with framebuffer info.
void render_primitive_viewer(GS& gs) {
    ImGui::Begin("Primitive Viewer", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    // Retrieve a copy of the primitives.
    std::vector<Primitive> primitives = gs.get_queued_primitives();

    ImGui::Text("Queued Primitives: %zu", primitives.size());
    ImGui::Separator();

    // Display some framebuffer info.
    ImGui::Text("Framebuffer 1: %dx%d, Format: %d, FBW: %d", 
                gs.framebuffer1.width, gs.framebuffer1.height, 
                gs.framebuffer1.format, gs.framebuffer1.fbw);
    ImGui::Separator();

    // Retrieve scissor values from the GS registers (assumed to be in SCISSOR_1).
    uint64_t scissor = gs.gs_registers[0x40];
    int scax0 = scissor & 0x7ff;
    int scax1 = (scissor >> 16) & 0x7ff;
    int scay0 = (scissor >> 32) & 0x7ff;
    int scay1 = (scissor >> 48) & 0x7ff;

    for (size_t i = 0; i < primitives.size(); ++i) {
        const Primitive& prim = primitives[i];

        std::string prim_type_str;
        switch (prim.type) {
            case PrimitiveType::Point:      prim_type_str = "Point"; break;
            case PrimitiveType::Line:       prim_type_str = "Line"; break;
            case PrimitiveType::Triangle:   prim_type_str = "Triangle"; break;
            case PrimitiveType::Sprite:     prim_type_str = "Sprite"; break;
            default:                        prim_type_str = "Unknown"; break;
        }

        ImGui::PushID(static_cast<int>(i));
        if (ImGui::CollapsingHeader((prim_type_str + " #" + std::to_string(i)).c_str())) {
            ImGui::Text("Vertex Count: %zu", prim.vertices.size());

            ImVec4 avg_color = compute_average_color(prim.vertices);
            ImGui::ColorButton("Color", avg_color);

            for (size_t v = 0; v < prim.vertices.size() && v < 10; ++v) {
                const Vertex& vert = prim.vertices[v];
                std::ostringstream oss;
                oss << "V" << v << ": ("
                    << std::fixed << std::setprecision(1)
                    << vert.x << ", " << vert.y << ", " << vert.z << ")"
                    << " Color: 0x" << std::hex << std::setw(8) << std::setfill('0') << vert.color;
                ImGui::Text("%s", oss.str().c_str());
            }

            // *** Dynamic Preview Area ***
            float fb_width = static_cast<float>(gs.framebuffer1.width);
            float fb_height = static_cast<float>(gs.framebuffer1.height);
            if (fb_width > 0 && fb_height > 0) {
                float available_width = ImGui::GetContentRegionAvail().x;

                // Scale to fit the available width while maintaining aspect ratio
                float scale = available_width / fb_width;
                float preview_w = available_width;
                float preview_h = fb_height * scale;

                // Limit preview height to prevent excessive stretching
                if (preview_h > 400.0f) { // Arbitrary max height to avoid oversized previews
                    preview_h = 400.0f;
                    scale = preview_h / fb_height;
                }

                // Begin child window for the preview
                ImGui::Text("Preview:");
                ImGui::BeginChild("PreviewCanvas", ImVec2(preview_w, preview_h), true);
                ImDrawList* draw_list = ImGui::GetWindowDrawList();
                ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();

                // ** Centered Framebuffer **
                float fb_draw_w = fb_width * scale;
                float fb_draw_h = fb_height * scale;
                float fb_offset_x = canvas_p0.x + (preview_w - fb_draw_w) * 0.5f;
                float fb_offset_y = canvas_p0.y + (preview_h - fb_draw_h) * 0.5f;

                // Draw a dark background
                draw_list->AddRectFilled(canvas_p0, 
                                         ImVec2(canvas_p0.x + preview_w, canvas_p0.y + preview_h), 
                                         IM_COL32(50, 50, 50, 255));

                // Draw the framebuffer boundary (Red)
                draw_list->AddRect(ImVec2(fb_offset_x, fb_offset_y),
                                   ImVec2(fb_offset_x + fb_draw_w, fb_offset_y + fb_draw_h),
                                   IM_COL32(255, 0, 0, 255), 0.0f, 0, 3.0f);

                // Debug Info
                ImGui::Text("FB Pos: (%.1f, %.1f) Size: (%.1f x %.1f)", 
                            fb_offset_x, fb_offset_y, fb_draw_w, fb_draw_h);

                // ** Transform Function **
                auto transform = [=](const ImVec2& pos) -> ImVec2 {
                    return ImVec2(pos.x * scale + fb_offset_x, pos.y * scale + fb_offset_y);
                };

                // ** Draw Primitives with Scissor Test **
                if (prim.type == PrimitiveType::Triangle && prim.vertices.size() >= 3) {
                    ImVec2 p0 = transform(ImVec2(prim.vertices[0].x, prim.vertices[0].y));
                    ImVec2 p1 = transform(ImVec2(prim.vertices[1].x, prim.vertices[1].y));
                    ImVec2 p2 = transform(ImVec2(prim.vertices[2].x, prim.vertices[2].y));


                    draw_list->AddTriangle(p0, p1, p2, IM_COL32(255, 255, 255, 255), 2.0f);
                }
                else if (prim.type == PrimitiveType::Sprite && prim.vertices.size() >= 2) {
                    ImVec2 p0 = transform(ImVec2(prim.vertices[0].x, prim.vertices[0].y));
                    ImVec2 p1 = transform(ImVec2(prim.vertices[1].x, prim.vertices[1].y));

                    draw_list->AddRect(p0, p1, IM_COL32(255, 255, 255, 255), 0.0f, 0, 2.0f);
                }
                else if (prim.type == PrimitiveType::Line && prim.vertices.size() >= 2) {
                    ImVec2 p0 = transform(ImVec2(prim.vertices[0].x, prim.vertices[0].y));
                    ImVec2 p1 = transform(ImVec2(prim.vertices[1].x, prim.vertices[1].y));

                   
                    draw_list->AddLine(p0, p1, IM_COL32(255, 255, 255, 255), 2.0f);
                }
                else if (prim.type == PrimitiveType::Point && !prim.vertices.empty()) {
                    ImVec2 p0 = transform(ImVec2(prim.vertices[0].x, prim.vertices[0].y));

                    draw_list->AddCircleFilled(p0, 3.0f, IM_COL32(255, 255, 255, 255));
                }
                ImGui::EndChild();
            }
        }
        ImGui::PopID();
    }

    ImGui::End();
}

void ImGui_Neo2::run(int argc, char **argv)
{
    Disassembler disassembler;

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

#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100 (WebGL 1.0)
    const char* glsl_version = "#version 100";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#elif defined(IMGUI_IMPL_OPENGL_ES3)
    // GL ES 3.0 + GLSL 300 es (WebGL 2.0)
    const char* glsl_version = "#version 300 es";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#elif defined(__APPLE__)
    // GL 3.2 Core + GLSL 150
    const char* glsl_version = "#version 150";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG); // Always required on Mac
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#endif

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    // Create window with SDL_Renderer graphics context
    Uint32 window_flags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN | SDL_WINDOW_HIGH_PIXEL_DENSITY;
    SDL_Window* window = SDL_CreateWindow("Neo2 - ImGui + SDL3", 1280, 720, window_flags);
    if (window == nullptr)
    {
        printf("Error: SDL_CreateWindow(): %s\n", SDL_GetError());
        return;
    }
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (gl_context == nullptr)
    {
        printf("Error: SDL_GL_CreateContext(): %s\n", SDL_GetError());
        return;
    }

    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync
    
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    SDL_ShowWindow(window);

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
    ImGui_ImplSDL3_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(glsl_version);

    this->bus.gs.opengl_.init(640, 480);
    this->bus.gs.set_render_mode(RenderMode::OpenGL);

    // Our state
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

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
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        ImGuiViewport* viewport = ImGui::GetMainViewport();

        GLuint texID = 0;
        // Get current framebuffer dimensions
        int fbWidth  = this->bus.gs.framebuffer1.width;
        int fbHeight = this->bus.gs.framebuffer1.height;

        // If the framebuffer size has changed, update previous values and (optionally) reinitialize textures
        if (fbWidth != prev_fb_width || fbHeight != prev_fb_height) {
                    prev_fb_width = this->bus.gs.framebuffer1.width;
                    prev_fb_height = this->bus.gs.framebuffer1.height;

                    texID = this->bus.gs.opengl_.getFrameTexture();
                    glBindTexture(GL_TEXTURE_2D, texID);
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, static_cast<int>(this->bus.gs.framebuffer1.width), static_cast<int>(this->bus.gs.framebuffer1.height), 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    glPixelStorei(GL_UNPACK_ROW_LENGTH, this->bus.gs.framebuffer1.fbw);
                    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, this->bus.gs.framebuffer1.width, this->bus.gs.framebuffer1.height, GL_RGBA, GL_UNSIGNED_BYTE, this->bus.gs.vram);
                    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
                    glBindTexture(GL_TEXTURE_2D, 0);
                }

        if (this->bus.gs.render_mode == RenderMode::Software)
        {
            texID = this->bus.gs.opengl_.getFrameTexture();
            glBindTexture(GL_TEXTURE_2D, texID);
            glPixelStorei(GL_UNPACK_ROW_LENGTH, this->bus.gs.framebuffer1.fbw);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, this->bus.gs.framebuffer1.width, this->bus.gs.framebuffer1.height, GL_RGBA, GL_UNSIGNED_BYTE, this->bus.gs.vram);
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        // Use fixed native dimensions for display
        const int nativeWidth  = 640;
        const int nativeHeight = 480;

        // Get window size from ImGui IO
        float windowWidth  = io.DisplaySize.x;
        float windowHeight = io.DisplaySize.y;

        // Assume your menubar and status bar each use the height of one frame
        float menubarHeight  = ImGui::GetFrameHeight();
        float statusbarHeight = ImGui::GetFrameHeight();

        // Compute the available area for the background (between menubar and status bar)
        float availableWidth  = windowWidth;
        float availableHeight = windowHeight - menubarHeight - statusbarHeight;

        // Calculate the maximum integer scale factor that fits in the available area
        int scaleFactor = std::max(1, (int)std::min( availableWidth  / (float)nativeWidth,
                                                        availableHeight / (float)nativeHeight ));

        // Compute the scaled dimensions based on the native 640x480 resolution
        int scaledWidth  = nativeWidth  * scaleFactor;
        int scaledHeight = nativeHeight * scaleFactor;

        // Center the scaled image in the available area
        float posX = (availableWidth - scaledWidth) * 0.5f;
        float posY = menubarHeight + (availableHeight - scaledHeight) * 0.5f;

        // Retrieve the current framebuffer texture ID (adjust according to your render mode)
        texID = bus.gs.opengl_.getFrameTexture(); // or your method for software mode

        // Draw the emulator “screen” as a background image using the native 640x480 dimensions (scaled up)
        ImDrawList* bgDrawList = ImGui::GetBackgroundDrawList();
        bgDrawList->AddImage((ImTextureID)(texID),
                            ImVec2(posX, posY),
                            ImVec2(posX + scaledWidth, posY + scaledHeight));

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
            std::string cur_render_mode = "Current Render Mode: ";
            if (this->bus.gs.render_mode == RenderMode::Software) {
                cur_render_mode += "Software";
            }
            else if (this->bus.gs.render_mode == RenderMode::OpenGL) {
                cur_render_mode += "OpenGL";
            }
            if (ImGui::Button(cur_render_mode.c_str())) {
                if (this->bus.gs.render_mode == RenderMode::Software) {
                    this->bus.gs.set_render_mode(RenderMode::OpenGL);
                } else if (this->bus.gs.render_mode == RenderMode::OpenGL) {
                    this->bus.gs.set_render_mode(RenderMode::Software);
                }
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
            render_textures(bus.gs, zoom_factor);
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
                    std::thread ee_thread([this, &is_running, &breakpoints, &stop_requested, &status_text, &status_color, &scheduler, &gs_vblank_task_id, &vblank_end_id]() {
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
                                    this->ee.execute_cycles(target_cycles, nullptr);
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
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(0.05f, 0.05f, 0.05f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        if (draw == true)
        {
            if (this->bus.gs.render_mode == RenderMode::OpenGL) this->bus.gs.opengl_.updateFromVram(this->bus.gs.vram, this->bus.gs.framebuffer1.width, this->bus.gs.framebuffer1.height, this->bus.gs.framebuffer1.fbw);
            this->bus.gs.batch_draw();
            draw = false;
        }

        SDL_GL_SwapWindow(window);
    }
#ifdef __EMSCRIPTEN__
    EMSCRIPTEN_MAINLOOP_END;
#endif

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DestroyContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
