#pragma once
#include <iostream>
#include <vector>
#include <glad/gl.h>  // or your chosen OpenGL loader header
#include <vertex.hh>

// Simple framebuffer class for ImGui/OpenGL integration.
class FrameBuffer
{
public:
    FrameBuffer(float width, float height);
    ~FrameBuffer();

    void init(float width, float height);

    unsigned int getFrameTexture();
    void RescaleFrameBuffer(float width, float height);
    void Bind() const;
    void Unbind() const;

    // These are used for drawing with the VAO/VBO pipeline.
    unsigned int vao = 0;
    unsigned int vbo = 0;
    unsigned int shader_program = 0;

    void finish_queue() const;
    void draw_test();
    void new_tex(uint32_t width, uint32_t height, uint32_t fbw);
	void draw_point_opengl(const Vertex& vertex, uint32_t width, uint32_t height, uint64_t scissor);
	void draw_triangle_opengl(const std::vector<Vertex>& vertices, uint32_t width, uint32_t height, uint64_t scissor);
    void draw_sprite_opengl(const std::vector<Vertex>& vertices, uint32_t width, uint32_t height, uint64_t scissor);
	void updateFromVram(const uint32_t* vram, int width, int height, int fbw);

private:
    unsigned int fbo;
    unsigned int texture;
    unsigned int rbo;
};
