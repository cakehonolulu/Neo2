#include <frontends/imgui/sdl3_gpu_renderer.h>
#include <sstream>
#include <iostream>
#include <cstddef>
#include <SDL3/SDL.h>
#include <log/log.hh>

SDLGPURenderer::SDLGPURenderer(float width, float height)
{
}

void SDLGPURenderer::init(SDL_GPUDevice *gpu_device_, SDL_GPUTexture *emu_texture_, SDL_GPUCommandBuffer *command_buffer_,
                       SDL_GPUGraphicsPipeline *hw_pipeline_, SDL_GPUBuffer *hw_vertex_buffer_)
{
    gpu_device = gpu_device_;
    emu_texture = emu_texture_;
    command_buffer = command_buffer_;
    hw_pipeline = hw_pipeline_;
    hw_vertex_buffer = hw_vertex_buffer_;
}

void SDLGPURenderer::UploadHWVertexData(const std::vector<Vertex> &vertices)
{
}

void SDLGPURenderer::RenderHWToEmuTexture(const std::vector<Vertex> &vertex_buffer)
{
}


SDLGPURenderer::~SDLGPURenderer()
{
}
