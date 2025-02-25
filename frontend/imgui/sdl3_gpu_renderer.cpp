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


SDL_GPUShader *LoadCompiledShader(SDL_GPUDevice *gpu_device, const char *filePath, SDL_GPUShaderStage stage)
{
    size_t codeSize = 0;
    void *code = SDL_LoadFile(filePath, &codeSize);
    if (!code)
    {
        printf("LoadCompiledShader: Failed to load shader file '%s': %s\n", filePath, SDL_GetError());
        return nullptr;
    }

    SDL_GPUShaderFormat backendFormats = SDL_GetGPUShaderFormats(gpu_device);

    SDL_GPUShaderCreateInfo shaderInfo;
    SDL_zero(shaderInfo);
    shaderInfo.code = reinterpret_cast<const Uint8 *>(code);
    shaderInfo.code_size = codeSize;
    shaderInfo.entrypoint = "main";
    shaderInfo.stage = stage;
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
        printf("Unrecognized backend shader format!\n");
        return NULL;
    }

    // Create the shader.
    SDL_GPUShader *shader = SDL_CreateGPUShader(gpu_device, &shaderInfo);
    if (!shader)
    {
        printf("LoadCompiledShader: Failed to create shader from file '%s': %s\n", filePath, SDL_GetError());
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
    shaderInfo.code = reinterpret_cast<const Uint8 *>(source);
    shaderInfo.entrypoint = "main";
    shaderInfo.format = SDL_GPU_SHADERFORMAT_DXIL;
    shaderInfo.stage = stage;
    shaderInfo.num_samplers = 1;
    shaderInfo.num_uniform_buffers = 1;

    SDL_GPUShader *shader = SDL_CreateGPUShader(gpu_device, &shaderInfo);
    if (!shader)
    {
        printf("Failed to create shader: %s\n", SDL_GetError());
    }
    return shader;
}

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
        printf("CreatePipeline: Failed to create graphics pipeline: %s\n", SDL_GetError());
    }
    return pipeline;
}
