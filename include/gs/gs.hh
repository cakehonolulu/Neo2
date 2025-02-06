#pragma once
#include <reg.hh>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>
#include <queue>
#include <mutex>
#include <vertex.hh>
#include <frontends/imgui/framebuffer.h>

// New structures for GS_Framebuffer and vertex buffer
struct GS_Framebuffer {
    uint32_t* data;
    uint32_t width;
    uint32_t height;
    uint32_t format;
    uint32_t fbw;
};

enum class PrimitiveType {
    Point,
    Line,
    Triangle = 3,
    Sprite = 6,
    None = 0xF
};

static const char* prims[] = { "", "", "", "Triangle", "", "", "Sprite"};

struct Primitive {
    PrimitiveType type;
    std::vector<Vertex> vertices;
};

enum class RenderMode {
    Software,
    OpenGL
};

class GS {
public:
    struct Texture {
        uint32_t address;
        uint32_t width;
        uint32_t height;
        uint32_t format;
        std::string name;
    };

    // Constants for register addresses
    static constexpr uint32_t BASE_ADDRESS = 0x12000000;
    static constexpr uint32_t CSR_ADDRESS = 0x12001000;
    static constexpr uint32_t IMR_ADDRESS = 0x12001010;
    static constexpr uint32_t BUSDIR_ADDRESS = 0x12001040;
    static constexpr uint32_t SIGLBLID_ADDRESS = 0x12001080;

    static constexpr const char* REGISTER_NAMES[19] = {
        "PMODE", "SMODE1", "SMODE2", "SRFSH", "SYNCH1", "SYNCH2", "SYNCV", 
        "DISPFB1", "DISPLAY1", "DISPFB2", "DISPLAY2", "EXTBUF", "EXTDATA", 
        "EXTWRITE", "BGCOLOR", "GS_CSR", "GS_IMR", "BUSDIR", "SIGLBLID"
    };

    static constexpr const char* INTERNAL_REGISTER_NAMES[57] = {
        "PRIM", "RGBAQ", "ST", "UV", "XYZF2", "XYZ2", "TEX0_1", "TEX0_2", 
        "CLAMP_1", "CLAMP_2", "FOG", "", "XYZF3", "XYZ3", "TEX1_1", "TEX1_2", 
        "TEX2_1", "TEX2_2", "XYOFFSET_1", "XYOFFSET_2", "PRMODECONT", "PRMODE", 
        "TEXCLUT", "", "", "", "", "", "TEXA", "", "FOGCOL", "", "TEXFLUSH", 
        "SCISSOR_1", "SCISSOR_2", "ALPHA_1", "ALPHA_2", "DIMX", "DTHE", "COLCLAMP", 
        "TEST_1", "TEST_2", "PABE", "FBA_1", "FBA_2", "FRAME_1", "FRAME_2", 
        "ZBUF_1", "ZBUF_2", "BITBLTBUF", "TRXPOS", "TRXREG", "TRXDIR", "HWREG", 
        "SIGNAL", "FINISH", "LABEL"
    };

    static constexpr std::size_t VRAM_SIZE = 4 * 1024 * 1024; // 4MB VRAM size

    GS();
    ~GS();

    FrameBuffer opengl_;

    // Methods to handle register reads and writes
    uint64_t read(uint32_t address);
    void write(uint32_t address, uint64_t value);

    void simul_vblank();
    void untog_vblank();


    // Method to handle incoming GIF data
    void write_gif_data(uint64_t data);
    void write_internal_reg(uint8_t reg, uint64_t data);
    void write_packed_gif_data(uint8_t reg, uint128_t data, uint32_t q);

    void set_bitbltbuf(uint64_t value);
    void set_trxpos(uint64_t value);
    void set_trxreg(uint64_t value);
    void set_trxdir(uint64_t value);
    void blit_vram();
    void transfer_vram();

    uint32_t* vram; // VRAM pointer

    uint64_t gs_privileged_registers[19]; // 19 privileged registers
    uint64_t gs_registers[0x63]; // 55 general registers

    void write_hwreg(uint64_t data);

    // Texture tracking
    void upload_texture(const Texture& texture);
    const std::vector<Texture>& get_textures() const;

    // New methods for handling PRIM selection and rendering
    void handle_prim_selection(uint64_t prim);

    // Renderer methods.
    // Software renderer.
    void draw_triangle_software(const std::vector<Vertex>& vertices);
    void draw_sprite_software(const std::vector<Vertex>& vertices);

    // OpenGL renderer.
    void draw_triangle_opengl(const std::vector<Vertex>& vertices, uint32_t width, uint32_t height, uint64_t scissor);
    void draw_sprite_opengl(const std::vector<Vertex>& vertices, uint32_t width, uint32_t height, uint64_t scissor);

    // Branching methods.
    void draw_triangle(const std::vector<Vertex>& vertices) {
        if (render_mode == RenderMode::Software)
            draw_triangle_software(vertices);
        else
            draw_triangle_opengl(vertices, framebuffer1.width, framebuffer1.height, gs_registers[0x40]);
    }

    void draw_sprite(const std::vector<Vertex>& vertices) {
        if (render_mode == RenderMode::Software)
            draw_sprite_software(vertices);
        else
            draw_sprite_opengl(vertices, framebuffer1.width, framebuffer1.height, gs_registers[0x40]);
    }

    RenderMode render_mode = RenderMode::Software;
    void set_render_mode(RenderMode mode) { render_mode = mode; }

    // Methods for handling framebuffer changes
    void update_framebuffer(uint32_t frame, uint32_t width, uint32_t height, uint32_t format);

    GS_Framebuffer framebuffer1;
    GS_Framebuffer framebuffer2;

    // Add the add_vertex method
    void add_vertex(uint64_t data, bool vertex_kick);
    void batch_draw();

    mutable std::mutex prim_queue_mutex;
    void dump_first_non_empty_queue(const std::string& filename);
    
    std::vector<Primitive> get_queued_primitives() const {
        std::lock_guard<std::mutex> lock(prim_queue_mutex);
        return std::vector<Primitive>(prim_queue.begin(), prim_queue.end());
    }

private:
    uint64_t bitbltbuf = 0;
    uint64_t trxpos = 0;
    uint64_t trxreg = 0;
    uint64_t trxdir = 0;
    uint64_t hwreg = 0;

    // 50h BLITBLTBUF
    uint32_t source_base_pointer = 0;
    uint32_t source_buffer_width = 0;
    uint32_t source_format = 0;
    uint32_t destination_base_pointer = 0;
    uint32_t destination_buffer_width = 0;
    uint32_t destination_format = 0;

    // 51h TRXPOS
    uint32_t source_rectangle_x = 0;
    uint32_t source_rectangle_y = 0;
    uint32_t destination_rectangle_x = 0;
    uint32_t destination_rectangle_y = 0;
    uint32_t transmission_order = 0;

    // 52h TRXREG
    uint32_t transmission_area_pixel_width = 0;
    uint32_t transmission_area_pixel_height = 0;

    // 53h TRXDIR
    uint32_t transmission_direction = 0;

    // GIF->VRAM aux.
    uint32_t source_x = 0;
    uint32_t source_y = 0;
    uint32_t destination_x = 0;
    uint32_t destination_y = 0;

    std::vector<Texture> textures; // List of uploaded textures

    std::deque<Primitive> prim_queue;

    PrimitiveType current_primitive = PrimitiveType::None;

    std::vector<Vertex> vertex_buffer; // Declare vertex_buffer
    uint64_t current_prim;
    uint32_t vertex_count;

    uint16_t interpolate(int32_t x, int32_t x1, int32_t y1, int32_t x2, int32_t y2);
    void draw_pixel(int32_t x, int32_t y, uint32_t color, uint32_t z, bool alpha_blend);
};
