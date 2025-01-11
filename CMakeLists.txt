cmake_minimum_required(VERSION 3.23)
project(Neo2 LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_FLAGS_DEBUG "-Wall -Wextra -g")
set(CMAKE_C_FLAGS_MINSIZEREL "-Wall -Wextra -O0")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-Wall -Wextra -g -O2")
set(CMAKE_C_FLAGS_RELEASE "-Wall -Wextra -O2")

if (ASAN)
    add_compile_options(-fsanitize=address,leak,undefined,pointer-compare,pointer-subtract -g)
    add_link_options(-fsanitize=address,leak,undefined,pointer-compare,pointer-subtract)
endif()

add_compile_options(-g -Wall -Wextra -std=c++2b)

find_package(LLVM REQUIRED CONFIG)
include_directories(${LLVM_INCLUDE_DIRS})
add_definitions(${LLVM_DEFINITIONS})

add_executable(neo2)

add_subdirectory(src)

add_subdirectory(frontend)

add_subdirectory(external/argparse)

target_include_directories(neo2 PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_include_directories(neo2 PRIVATE ${PROJECT_SOURCE_DIR}/external/argparse/include)
target_link_libraries(neo2 PRIVATE LLVM)
target_link_options(neo2 PRIVATE -Wl,--export-dynamic)
