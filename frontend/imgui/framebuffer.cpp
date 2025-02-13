#include <frontends/imgui/framebuffer.h>

#include <sstream>
#include <iostream>
#include <cstddef> // for offsetof
#include <SDL3/SDL.h>
#include <glad/gl.h>  // Include glad for OpenGL functions
#include <log/log.hh>

#define GL_CHECK_ERROR() \
    do { \
        GLenum err; \
        while ((err = glGetError()) != GL_NO_ERROR) { \
            std::cerr << "OpenGL error: " << err << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
        } \
    } while (0)

FrameBuffer::FrameBuffer(float width, float height)
{
}

void FrameBuffer::init(float width, float height) {
    
    if (!gladLoadGL((GLADloadfunc)SDL_GL_GetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return;
    }
    GL_CHECK_ERROR();

    // Create framebuffer.
    glGenFramebuffers(1, &fbo);
    GL_CHECK_ERROR();
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    GL_CHECK_ERROR();

    // Create texture to attach as the color attachment.
    glGenTextures(1, &texture);
    GL_CHECK_ERROR();
    glBindTexture(GL_TEXTURE_2D, texture);
    GL_CHECK_ERROR();
    // Using GL_RGB for internal format and format here (adjust if needed).
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, static_cast<int>(width), static_cast<int>(height), 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    GL_CHECK_ERROR();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    GL_CHECK_ERROR();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    GL_CHECK_ERROR();
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);
    GL_CHECK_ERROR();

    // Create a renderbuffer object for depth and stencil attachment.
    glGenRenderbuffers(1, &rbo);
    GL_CHECK_ERROR();
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    GL_CHECK_ERROR();
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, static_cast<int>(width), static_cast<int>(height));
    GL_CHECK_ERROR();
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
    GL_CHECK_ERROR();

    // Create the VAO and VBO for rendering (for example, for drawing primitives into the framebuffer).
    glGenVertexArrays(1, &vao);
    GL_CHECK_ERROR();
    glBindVertexArray(vao);
    GL_CHECK_ERROR();

    glGenBuffers(1, &vbo);
    GL_CHECK_ERROR();
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    GL_CHECK_ERROR();
    // Pre-allocate space for 1024 vertices (adjust as needed).
    glBufferData(GL_ARRAY_BUFFER, 1024 * sizeof(Vertex), nullptr, GL_DYNAMIC_DRAW);
    GL_CHECK_ERROR();

    // Set up attribute pointers.
    // Attribute 0: Position (vec3)
    glEnableVertexAttribArray(0);
    GL_CHECK_ERROR();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, x));
    GL_CHECK_ERROR();

    // Attribute 0: Position (vec3)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, x));

    // Attribute 1: Color (as an unsigned int)
    // Use the integer version so that the 32-bit value is passed unmodified.
    glEnableVertexAttribArray(1);
    glVertexAttribIPointer(1, 1, GL_UNSIGNED_INT, sizeof(Vertex), (void*)offsetof(Vertex, color));

    // Attribute 2: Texture Coordinates (vec2)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, u));


    glBindBuffer(GL_ARRAY_BUFFER, 0);
    GL_CHECK_ERROR();
    glBindVertexArray(0);
    GL_CHECK_ERROR();

    // Check framebuffer completeness.
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    GL_CHECK_ERROR();

    // Now compile our shaders.
    // --- Vertex Shader Source ---
    const char* vertexShaderSource = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        // Read the raw 32-bit color.
        layout(location = 1) in uint aColorRaw;
        out vec4 vertexColor;
        
        // Instead of doing manual division, we use an orthographic projection.
        uniform mat4 uProjection;
        
        vec4 unpackColor(uint color) {
            float r = float((color >> 24u) & 0xFFu) / 255.0;
            float g = float((color >> 16u) & 0xFFu) / 255.0;
            float b = float((color >> 8u) & 0xFFu) / 255.0;
            float a = float(color & 0xFFu) / 255.0;
            return vec4(r, g, b, a);
        }
        
        void main() {
            gl_Position = uProjection * vec4(aPos, 1.0);
            vertexColor = unpackColor(aColorRaw);
        }
    )";

    // --- Fragment Shader Source ---
    const char* fragmentShaderSource = R"(
        #version 330 core
        in vec4 vertexColor;
        out vec4 FragColor;
        void main() {
            FragColor = vertexColor;
        }
    )";

    // Helper lambda to compile a shader.
    auto compileShader = [](const char* source, GLenum shaderType) -> unsigned int {
        unsigned int shader = glCreateShader(shaderType);
        GL_CHECK_ERROR();
        glShaderSource(shader, 1, &source, NULL);
        GL_CHECK_ERROR();
        glCompileShader(shader);
        GL_CHECK_ERROR();
        int success;
        char infoLog[512];
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        GL_CHECK_ERROR();
        if (!success) {
            glGetShaderInfoLog(shader, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
        }
        return shader;
    };

    unsigned int vertexShader = compileShader(vertexShaderSource, GL_VERTEX_SHADER);
    unsigned int fragmentShader = compileShader(fragmentShaderSource, GL_FRAGMENT_SHADER);

    // Create and link the shader program.
    shader_program = glCreateProgram();
    GL_CHECK_ERROR();
    glAttachShader(shader_program, vertexShader);
    GL_CHECK_ERROR();
    glAttachShader(shader_program, fragmentShader);
    GL_CHECK_ERROR();
    glLinkProgram(shader_program);
    GL_CHECK_ERROR();
    int success;
    char infoLog[512];
    glGetProgramiv(shader_program, GL_LINK_STATUS, &success);
    GL_CHECK_ERROR();
    if (!success) {
        glGetProgramInfoLog(shader_program, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER_PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }
    // Delete shaders once linked.
    glDeleteShader(vertexShader);
    GL_CHECK_ERROR();
    glDeleteShader(fragmentShader);
    GL_CHECK_ERROR();

    // Unbind framebuffer, texture, renderbuffer.
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    GL_CHECK_ERROR();
    glBindTexture(GL_TEXTURE_2D, 0);
    GL_CHECK_ERROR();
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    GL_CHECK_ERROR();
}

FrameBuffer::~FrameBuffer()
{
    glDeleteFramebuffers(1, &fbo);
    GL_CHECK_ERROR();
    glDeleteTextures(1, &texture);
    GL_CHECK_ERROR();
    glDeleteRenderbuffers(1, &rbo);
    GL_CHECK_ERROR();
    glDeleteVertexArrays(1, &vao);
    GL_CHECK_ERROR();
    glDeleteBuffers(1, &vbo);
    GL_CHECK_ERROR();
    glDeleteProgram(shader_program);
    GL_CHECK_ERROR();
}

unsigned int FrameBuffer::getFrameTexture()
{
    return texture;
}

void FrameBuffer::RescaleFrameBuffer(float width, float height)
{
    glBindTexture(GL_TEXTURE_2D, texture);
    GL_CHECK_ERROR();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, static_cast<int>(width), static_cast<int>(height), 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    GL_CHECK_ERROR();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    GL_CHECK_ERROR();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    GL_CHECK_ERROR();
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);
    GL_CHECK_ERROR();

    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    GL_CHECK_ERROR();
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, static_cast<int>(width), static_cast<int>(height));
    GL_CHECK_ERROR();
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
    GL_CHECK_ERROR();
}

void FrameBuffer::Bind() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    GL_CHECK_ERROR();
}

void FrameBuffer::Unbind() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    GL_CHECK_ERROR();
}

void FrameBuffer::finish_queue() const
{
    glUseProgram(0);
    GL_CHECK_ERROR();
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    GL_CHECK_ERROR();
    glBindVertexArray(0);
    GL_CHECK_ERROR();
    Unbind();
}

void FrameBuffer::updateFromVram(const uint32_t* vram, int width, int height, int fbw) {
    Bind();
    // Bind our texture.
    glBindTexture(GL_TEXTURE_2D, texture);
    GL_CHECK_ERROR();

    // Set the unpack row length so that OpenGL knows how many pixels per row
    // are in the source data. (This is needed if fbw is larger than width.)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, fbw);
    GL_CHECK_ERROR();

    // Update the texture data from the provided vram pointer.
    // We assume that the pixel data are in GL_RGBA format and each pixel is 4 bytes.
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, vram);
    GL_CHECK_ERROR();

    // Reset the unpack row length to default.
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    GL_CHECK_ERROR();
    
    glBindTexture(GL_TEXTURE_2D, 0);
    GL_CHECK_ERROR();

    Unbind();
    GL_CHECK_ERROR();
}

void FrameBuffer::draw_point_opengl(const Vertex& vertex, uint32_t width, uint32_t height, uint64_t scissor) {
    // Save current viewport
    GLint oldViewport[4];
    glGetIntegerv(GL_VIEWPORT, oldViewport);

    // Set new viewport
    glViewport(0, 0, width, height);

    // Bind the framebuffer
    Bind();

    glEnable(GL_SCISSOR_TEST);

    int scax0 = scissor & 0x7FF;
    int scax1 = (scissor >> 16) & 0x7FF;
    int scay0 = (scissor >> 32) & 0x7FF;
    int scay1 = (scissor >> 48) & 0x7FF;

    int ogl_scay0 = height - scay1 - 1;
    int ogl_scay1 = height - scay0;
    int ogl_height = ogl_scay1 - ogl_scay0;

    glScissor(scax0, scay0, scax1 - scax0 + 1, ogl_height);

    // Bind the VAO
    glBindVertexArray(vao);

    // Update the VBO with vertex data
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vertex), &vertex);

    // Use the shader program
    glUseProgram(shader_program);

    // Compute and set the orthographic projection matrix
    float left = 0.0f;
    float right = static_cast<float>(width);
    float top = 0.0f;
    float bottom = static_cast<float>(height);
    float near = -1.0f;
    float far = 1.0f;
    float ortho[16] = {
         2.0f / (right - left),  0.0f,                0.0f,  0.0f,
         0.0f,                  -2.0f / (bottom - top), 0.0f,  0.0f,
         0.0f,                   0.0f,               -2.0f / (far - near),  0.0f,
        -(right + left) / (right - left), (bottom + top) / (bottom - top), -(far + near) / (far - near), 1.0f
    };

    GLint projLoc = glGetUniformLocation(shader_program, "uProjection");
    if (projLoc != -1) {
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, ortho);
    } else {
        std::cerr << "Warning: uProjection uniform not found!" << std::endl;
    }

    // Draw the vertex as a point.
    glDrawArrays(GL_POINTS, 0, 1);

    // Unbind
    glUseProgram(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glDisable(GL_SCISSOR_TEST);
    Unbind();

    // Restore original viewport
    glViewport(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]);
}


void FrameBuffer::draw_triangle_opengl(const std::vector<Vertex>& vertices, uint32_t width, uint32_t height, uint64_t scissor) {
    // Save current viewport
    GLint oldViewport[4];
    glGetIntegerv(GL_VIEWPORT, oldViewport);

    // Set new viewport
    glViewport(0, 0, width, height);

    // Bind the framebuffer.
    Bind();

    glEnable(GL_SCISSOR_TEST);

    int scax0 = scissor & 0x7FF;
    int scax1 = (scissor >> 16) & 0x7FF;
    int scay0 = (scissor >> 32) & 0x7FF;
    int scay1 = (scissor >> 48) & 0x7FF;

    int ogl_scay0 = height - scay1 - 1;
    int ogl_scay1 = height - scay0;
    int ogl_height = ogl_scay1 - ogl_scay0;

    glScissor(scax0, scay0, scax1 - scax0 + 1, ogl_height);

    // Bind the VAO.
    glBindVertexArray(vao);

    // Update the VBO with vertex data.
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(Vertex), vertices.data());

    // Use the shader program.
    glUseProgram(shader_program);

    // Compute and set the orthographic projection matrix
    float left = 0.0f;
    float right = static_cast<float>(width);
    float top = 0.0f;
    float bottom = static_cast<float>(height);
    float near = -1.0f;
    float far = 1.0f;
    float ortho[16] = {
         2.0f / (right - left),  0.0f,                0.0f,  0.0f,
         0.0f,                  -2.0f / (bottom - top), 0.0f,  0.0f,
         0.0f,                   0.0f,               -2.0f / (far - near),  0.0f,
        -(right + left) / (right - left), (bottom + top) / (bottom - top), -(far + near) / (far - near), 1.0f
    };

    GLint projLoc = glGetUniformLocation(shader_program, "uProjection");
    if (projLoc != -1) {
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, ortho);
    } else {
        std::cerr << "Warning: uProjection uniform not found!" << std::endl;
    }

    // Draw the vertices as triangles.
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices.size()));

    // Unbind.
    glUseProgram(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glDisable(GL_SCISSOR_TEST);
    Unbind();

    // Restore original viewport
    glViewport(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]);
}



void FrameBuffer::draw_sprite_opengl(const std::vector<Vertex>& vertices, uint32_t width, uint32_t height, uint64_t scissor) {
    if (vertices.size() < 2) {
        Logger::error("Not enough vertices to draw sprite (need at least 2).");
        return;
    }

    // Save current viewport
    GLint oldViewport[4];
    glGetIntegerv(GL_VIEWPORT, oldViewport);

    // Set new viewport
    glViewport(0, 0, width, height);

    // Bind the framebuffer
    Bind();

    glEnable(GL_SCISSOR_TEST);

    int scax0 = scissor & 0x7FF;
    int scax1 = (scissor >> 16) & 0x7FF;
    int scay0 = (scissor >> 32) & 0x7FF;
    int scay1 = (scissor >> 48) & 0x7FF;

    int ogl_scay0 = height - scay1 - 1;
    int ogl_scay1 = height - scay0;
    int ogl_height = ogl_scay1 - ogl_scay0;

    glScissor(scax0, scay0, scax1 - scax0 + 1, ogl_height);

    // Bind the VAO
    glBindVertexArray(vao);

    // Update the VBO with vertex data
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    std::vector<Vertex> vertices_ = {
        {vertices[0].x, vertices[0].y, 0.0f, vertices[0].color, 0.0f, 0.0f}, // First triangle
        {vertices[1].x, vertices[0].y, 0.0f, vertices[0].color, 0.0f, 0.0f},
        {vertices[0].x, vertices[1].x, 0.0f, vertices[0].color, 0.0f, 0.0f},
        {vertices[1].x, vertices[1].y, 0.0f, vertices[0].color, 0.0f, 0.0f}  // Second triangle
    };

    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices_.size() * sizeof(Vertex), vertices_.data());

    // Use the shader program
    glUseProgram(shader_program);

    // Compute and set the orthographic projection matrix
    float left = 0.0f;
    float right = static_cast<float>(width);
    float top = 0.0f;
    float bottom = static_cast<float>(height);
    float near = -1.0f;
    float far = 1.0f;
    float ortho[16] = {
         2.0f / (right - left),  0.0f,                0.0f,  0.0f,
         0.0f,                  -2.0f / (bottom - top), 0.0f,  0.0f,
         0.0f,                   0.0f,               -2.0f / (far - near),  0.0f,
        -(right + left) / (right - left), (bottom + top) / (bottom - top), -(far + near) / (far - near), 1.0f
    };

    GLint projLoc = glGetUniformLocation(shader_program, "uProjection");
    if (projLoc != -1) {
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, ortho);
    } else {
        std::cerr << "Warning: uProjection uniform not found!" << std::endl;
    }

    // Draw the vertices as a triangle strip (2 triangles forming a rectangle)
    glDrawArrays(GL_TRIANGLE_STRIP, 0, static_cast<GLsizei>(vertices.size()));

    // Unbind
    glUseProgram(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glDisable(GL_SCISSOR_TEST);
    Unbind();

    // Restore original viewport
    glViewport(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]);
}


void FrameBuffer::draw_test() {
    Bind();
    GL_CHECK_ERROR();

    glBindTexture(GL_TEXTURE_2D, texture);
    GL_CHECK_ERROR();
    
    std::vector<Vertex> vertices = {
        { 0.0f,  0.5f, 0.0f, 0x12345678, 0.0f, 0.0f}, // Vertex 1: Position (x, y, z) and Color (r, g, b)
        {-0.5f, -0.5f, 0.0f, 0x12345678, 0.0f, 0.0f}, // Vertex 2: Position (x, y, z) and Color (r, g, b)
        { 0.5f, -0.5f, 0.0f, 0x12345678, 0.0f, 0.0f}  // Vertex 3: Position (x, y, z) and Color (r, g, b)
    };

    GL_CHECK_ERROR();
    glBindVertexArray(vao);
    GL_CHECK_ERROR();
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    GL_CHECK_ERROR();
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(Vertex), vertices.data());
    GL_CHECK_ERROR();
    glUseProgram(shader_program);
    GL_CHECK_ERROR();

    // For sprite rendering, if GL_QUADS is available, you could use that.
    // In core OpenGL profiles GL_QUADS is deprecated. You might want to draw two triangles.
    // For simplicity, if you have 4 vertices you can draw as a quad using GL_TRIANGLE_STRIP:
    glDrawArrays(GL_TRIANGLE_STRIP, 0, vertices.size());
    GL_CHECK_ERROR();

    glUseProgram(0);
    GL_CHECK_ERROR();
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    GL_CHECK_ERROR();
    glBindVertexArray(0);
    GL_CHECK_ERROR();
    glBindTexture(GL_TEXTURE_2D, 0);
    GL_CHECK_ERROR();
    Unbind();
    GL_CHECK_ERROR();
}