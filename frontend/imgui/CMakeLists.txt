set(OpenGL_GL_PREFERENCE GLVND)

find_package(OpenGL REQUIRED)
find_package(SDL3 REQUIRED)

set(EXTERNAL_SOURCE_DIR ${PROJECT_SOURCE_DIR}/external)

# Add ImGui source files
set(IMGUI_SOURCE_DIR ${EXTERNAL_SOURCE_DIR}/imgui)

set(IMGUI_SOURCES
        "${IMGUI_SOURCE_DIR}/imgui.h"
        "${IMGUI_SOURCE_DIR}/imgui.cpp"
        "${IMGUI_SOURCE_DIR}/imgui_draw.cpp"
        "${IMGUI_SOURCE_DIR}/imgui_widgets.cpp"
        "${IMGUI_SOURCE_DIR}/imgui_tables.cpp"
        "${IMGUI_SOURCE_DIR}/backends/imgui_impl_sdlrenderer3.cpp"
        "${IMGUI_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp"
        "${IMGUI_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
)

set(IMGUI_FILE_DIALOG_SOURCES
        "${EXTERNAL_SOURCE_DIR}/ImGuiFileDialog/ImGuiFileDialog.cpp"
        "${EXTERNAL_SOURCE_DIR}/ImGuiFileDialog/ImGuiFileDialog.h"
)

target_sources(neo2
        PRIVATE
        ${IMGUI_SOURCES}
        ${IMGUI_FILE_DIALOG_SOURCES}
        main.cpp
        imgui_neo2.cpp
        imgui_debug.cpp
        imgui_log.cpp
        imgui_exit.cpp)

# Add OpenGL, SDL2, and ImGui include files to compilation
target_include_directories(neo2 PUBLIC
        ${OPENGL_INCLUDE_DIRS}
        ${SDL3_INCLUDE_DIRS}
        ${IMGUI_SOURCE_DIR}
        ${IMGUI_SOURCE_DIR}/backends
        ${EXTERNAL_SOURCE_DIR}/ImGuiFileDialog
)

# Link the library to the main executable
target_link_libraries(neo2 PRIVATE ${OPENGL_LIBRARIES} ${SDL3_LIBRARIES})