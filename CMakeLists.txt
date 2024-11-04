cmake_minimum_required(VERSION 3.23)
project(Neo2 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_FLAGS_DEBUG "-Wall -Wextra -g")
set(CMAKE_C_FLAGS_MINSIZEREL "-Wall -Wextra -O0")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-Wall -Wextra -g -O2")
set(CMAKE_C_FLAGS_RELEASE "-Wall -Wextra -O2")

if (ASAN)
add_compile_options(-fsanitize=address -g)
add_link_options(-fsanitize=address)
endif()

add_compile_options(-g -Wall -Wextra -std=c++2b)

add_executable(neo2)

add_subdirectory(src)

add_subdirectory(frontend)

target_include_directories(neo2 PRIVATE ${PROJECT_SOURCE_DIR}/include)
