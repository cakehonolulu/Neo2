cmake_minimum_required(VERSION 3.23)
project(Neo2 LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_FLAGS_DEBUG "-Wall -Wextra -g")
set(CMAKE_C_FLAGS_MINSIZEREL "-Wall -Wextra -O0")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-Wall -Wextra -g -O2")
set(CMAKE_C_FLAGS_RELEASE "-Wall -Wextra -O2")

option(SANITIZER "Enable ASAN/LSAN/UBSAN" OFF)

if (SANITIZER)
    add_compile_options(-fsanitize=address,leak,undefined,pointer-compare,pointer-subtract -g)
    add_link_options(-fsanitize=address,leak,undefined,pointer-compare,pointer-subtract)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra -std=c++2b")

find_package(LLVM REQUIRED CONFIG)
include_directories(${LLVM_INCLUDE_DIRS})
add_definitions(${LLVM_DEFINITIONS})

find_package(GLEW REQUIRED) # Add this line to find GLEW
include_directories(${GLEW_INCLUDE_DIRS}) # Include GLEW headers

add_executable(neo2)

add_subdirectory(src)

add_subdirectory(frontend)

add_subdirectory(external/argparse)

target_include_directories(neo2 PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_include_directories(neo2 PRIVATE ${PROJECT_SOURCE_DIR}/external/argparse/include)
target_include_directories(neo2 PRIVATE ${PROJECT_SOURCE_DIR}/external/imgui_club/imgui_memory_editor)

target_link_libraries(neo2 PRIVATE LLVM)
target_link_libraries(neo2 PRIVATE ${GLEW_LIBRARIES}) # Link GLEW library
target_link_options(neo2 PRIVATE -Wl,--export-dynamic)
