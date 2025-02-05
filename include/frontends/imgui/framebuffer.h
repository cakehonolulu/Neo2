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

	void draw_triangle_opengl(const std::vector<Vertex>& vertices);
    void draw_sprite_opengl(const std::vector<Vertex>& vertices);
	void updateFromVram(const uint32_t* vram, int width, int height, int fbw);

private:
    unsigned int fbo;
    unsigned int texture;
    unsigned int rbo;
};
