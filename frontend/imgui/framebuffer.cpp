#include <frontends/imgui/framebuffer.h>

#include <sstream>
#include <iostream>
#include <cstddef> // for offsetof
#include <GL/glx.h>
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
    
    if (!gladLoadGL((GLADloadfunc)glXGetProcAddress)) {
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

    // Attribute 1: Color (vec4 stored as 4 unsigned bytes)
    glEnableVertexAttribArray(1);
    GL_CHECK_ERROR();
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Vertex), (void*)offsetof(Vertex, color));
    GL_CHECK_ERROR();

    // Attribute 2: Texture Coordinates (vec2)
    glEnableVertexAttribArray(2);
    GL_CHECK_ERROR();
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, u));
    GL_CHECK_ERROR();

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
        layout(location = 1) in vec4 aColor;
        out vec4 vertexColor;
        void main() {
            gl_Position = vec4(aPos, 1.0);
            vertexColor = aColor;
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

    // Unbind the texture.
    glBindTexture(GL_TEXTURE_2D, 0);
    GL_CHECK_ERROR();
    Unbind();
}

void FrameBuffer::draw_triangle_opengl(const std::vector<Vertex>& vertices) {
    /*Bind();
    GL_CHECK_ERROR();
   
    // Bind the texture.
    glBindTexture(GL_TEXTURE_2D, texture);
    GL_CHECK_ERROR();

    // Ensure our VAO is bound.
    glBindVertexArray(vao);
    GL_CHECK_ERROR();
   
    // Update the VBO with vertex data.
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    GL_CHECK_ERROR();
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(Vertex), vertices.data());
    GL_CHECK_ERROR();

    // Use our shader program.
    glUseProgram(shader_program);
    GL_CHECK_ERROR();

    // Draw the vertices as triangles.
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices.size()));
    GL_CHECK_ERROR();

    // Unbind resources.
    glUseProgram(0);
    GL_CHECK_ERROR();
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    GL_CHECK_ERROR();
    glBindVertexArray(0);
    GL_CHECK_ERROR();
    Unbind();*/
}


void FrameBuffer::draw_sprite_opengl(const std::vector<Vertex>& vertices) {
    /*Bind();
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
    Unbind();*/
}