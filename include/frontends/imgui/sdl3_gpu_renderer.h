#pragma once
#include <iostream>
#include <vector>
#include <SDL3/SDL_gpu.h>
#include <vertex.hh>

SDL_GPUShader *LoadCompiledShader(SDL_GPUDevice *gpu_device, SDL_GPUShaderStage stage);
SDL_GPUShader *CompileShader(SDL_GPUDevice *gpu_device, const char *source, SDL_GPUShaderStage stage);
SDL_GPUGraphicsPipeline *CreatePipeline(SDL_GPUDevice *gpu_device, SDL_Window *window, SDL_GPUShader *vertexShader,
                                        SDL_GPUShader *fragmentShader);

class SDLGPURenderer
{
public:
    SDLGPURenderer(float width, float height);
    ~SDLGPURenderer();

    void init(SDL_GPUDevice *gpu_device_, SDL_GPUTexture *emu_texture_, SDL_GPUCommandBuffer *command_buffer_,
              SDL_GPUGraphicsPipeline *hw_pipeline_, SDL_GPUBuffer *hw_vertex_buffer_);

    void UploadHWVertexData(const std::vector<Vertex> &vertices);
    void RenderHWToEmuTexture(const std::vector<Vertex> &vertex_buffer);

    SDL_GPUDevice *gpu_device;
    SDL_GPUTexture *emu_texture;
    SDL_GPUCommandBuffer *command_buffer;
    SDL_GPUGraphicsPipeline *hw_pipeline;
    SDL_GPUBuffer *hw_vertex_buffer;
};
