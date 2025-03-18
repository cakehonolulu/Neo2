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

if(WIN32 OR _WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /clang:-std=c++23")
file(DOWNLOAD "https://gist.githubusercontent.com/Nax/3c4e4de87fa53bd3659b8ca32c27a339/raw/d9d3e45d639c6e60b0fe440865807064688f3b97/elf.h"
                  "${CMAKE_SOURCE_DIR}/include/elf.h")
else()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra -std=c++2b")
endif()

find_package(LLVM REQUIRED CONFIG)
include_directories(${LLVM_INCLUDE_DIRS})
add_definitions(${LLVM_DEFINITIONS})
find_package(OpenSSL REQUIRED)
include_directories(${OPENSSL_INCLUDE_DIRS})

# Define custom commands to compile HLSL shaders to DXIL, DXBC, and SPIRV formats
set(SHADER_DIR ${CMAKE_SOURCE_DIR}/shaders)
set(COMPILED_SHADER_DIR ${CMAKE_SOURCE_DIR}/shaders)

file(MAKE_DIRECTORY ${COMPILED_SHADER_DIR})

add_custom_command(
    OUTPUT ${COMPILED_SHADER_DIR}/vertexShader.dxil
    COMMAND ${CMAKE_COMMAND} -E make_directory ${COMPILED_SHADER_DIR}
    COMMAND ${CMAKE_COMMAND} -E echo "Compiling vertexShader.hlsl to DXIL"
    COMMAND shadercross ${SHADER_DIR}/vertexShader.hlsl -o ${COMPILED_SHADER_DIR}/vertexShader.dxil -t vertex -d DXIL
    DEPENDS ${SHADER_DIR}/vertexShader.hlsl
    COMMENT "Compiling vertexShader.hlsl to DXIL"
    VERBATIM
)

add_custom_command(
    OUTPUT ${COMPILED_SHADER_DIR}/fragmentShader.dxil
    COMMAND ${CMAKE_COMMAND} -E make_directory ${COMPILED_SHADER_DIR}
    COMMAND ${CMAKE_COMMAND} -E echo "Compiling fragmentShader.hlsl to DXIL"
    COMMAND shadercross ${SHADER_DIR}/fragmentShader.hlsl -o ${COMPILED_SHADER_DIR}/fragmentShader.dxil -t fragment -d DXIL
    DEPENDS ${SHADER_DIR}/fragmentShader.hlsl
    COMMENT "Compiling fragmentShader.hlsl to DXIL"
    VERBATIM
)

add_custom_command(
    OUTPUT ${COMPILED_SHADER_DIR}/vertexShader.dxbc
    COMMAND ${CMAKE_COMMAND} -E make_directory ${COMPILED_SHADER_DIR}
    COMMAND ${CMAKE_COMMAND} -E echo "Compiling vertexShader.hlsl to DXBC"
    COMMAND shadercross ${SHADER_DIR}/vertexShader.hlsl -o ${COMPILED_SHADER_DIR}/vertexShader.dxbc -t vertex -d DXBC
    DEPENDS ${SHADER_DIR}/vertexShader.hlsl
    COMMENT "Compiling vertexShader.hlsl to DXBC"
    VERBATIM
)

add_custom_command(
    OUTPUT ${COMPILED_SHADER_DIR}/fragmentShader.dxbc
    COMMAND ${CMAKE_COMMAND} -E make_directory ${COMPILED_SHADER_DIR}
    COMMAND ${CMAKE_COMMAND} -E echo "Compiling fragmentShader.hlsl to DXBC"
    COMMAND shadercross ${SHADER_DIR}/fragmentShader.hlsl -o ${COMPILED_SHADER_DIR}/fragmentShader.dxbc -t fragment -d DXBC
    DEPENDS ${SHADER_DIR}/fragmentShader.hlsl
    COMMENT "Compiling fragmentShader.hlsl to DXBC"
    VERBATIM
)

add_custom_command(
    OUTPUT ${COMPILED_SHADER_DIR}/vertexShader.spv
    COMMAND ${CMAKE_COMMAND} -E make_directory ${COMPILED_SHADER_DIR}
    COMMAND ${CMAKE_COMMAND} -E echo "Compiling vertexShader.hlsl to SPIRV"
    COMMAND shadercross ${SHADER_DIR}/vertexShader.hlsl -o ${COMPILED_SHADER_DIR}/vertexShader.spv -t vertex -d SPIRV
    DEPENDS ${SHADER_DIR}/vertexShader.hlsl
    COMMENT "Compiling vertexShader.hlsl to SPIRV"
    VERBATIM
)

add_custom_command(
    OUTPUT ${COMPILED_SHADER_DIR}/fragmentShader.spv
    COMMAND ${CMAKE_COMMAND} -E make_directory ${COMPILED_SHADER_DIR}
    COMMAND ${CMAKE_COMMAND} -E echo "Compiling fragmentShader.hlsl to SPIRV"
    COMMAND shadercross ${SHADER_DIR}/fragmentShader.hlsl -o ${COMPILED_SHADER_DIR}/fragmentShader.spv -t fragment -d SPIRV
    DEPENDS ${SHADER_DIR}/fragmentShader.hlsl
    COMMENT "Compiling fragmentShader.hlsl to SPIRV"
    VERBATIM
)

add_custom_target(
    compile_shaders
    DEPENDS ${COMPILED_SHADER_DIR}/vertexShader.dxil
            ${COMPILED_SHADER_DIR}/fragmentShader.dxil
            ${COMPILED_SHADER_DIR}/vertexShader.dxbc
            ${COMPILED_SHADER_DIR}/fragmentShader.dxbc
            ${COMPILED_SHADER_DIR}/vertexShader.spv
            ${COMPILED_SHADER_DIR}/fragmentShader.spv
)

add_executable(neo2)

add_dependencies(neo2 compile_shaders)

add_subdirectory(src)

add_subdirectory(frontend)

add_subdirectory(external/argparse)

target_include_directories(neo2 PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_include_directories(neo2 PRIVATE ${PROJECT_SOURCE_DIR}/external/argparse/include)
target_include_directories(neo2 PRIVATE ${PROJECT_SOURCE_DIR}/external/imgui_club/imgui_memory_editor)

if(WIN32 OR _WIN32)
target_link_libraries(neo2 PRIVATE LLVMWindowsManifest LLVMXRay LLVMLibDriver LLVMDlltoolDriver LLVMTextAPIBinaryReader LLVMCoverage LLVMLineEditor LLVMX86TargetMCA LLVMX86Disassembler LLVMX86AsmParser LLVMX86CodeGen LLVMX86Desc LLVMX86Info LLVMOrcDebugging LLVMOrcJIT LLVMWindowsDriver LLVMMCJIT LLVMJITLink LLVMInterpreter LLVMExecutionEngine LLVMRuntimeDyld LLVMOrcTargetProcess LLVMOrcShared LLVMDWP LLVMDebugInfoLogicalView LLVMDebugInfoGSYM LLVMOption LLVMObjectYAML LLVMObjCopy LLVMMCA LLVMMCDisassembler LLVMLTO LLVMPasses LLVMHipStdPar LLVMCFGuard LLVMCoroutines LLVMipo LLVMVectorize LLVMLinker LLVMInstrumentation LLVMFrontendOpenMP LLVMFrontendOffloading LLVMFrontendOpenACC LLVMFrontendHLSL LLVMFrontendDriver LLVMExtensions LLVMDWARFLinkerParallel LLVMDWARFLinkerClassic LLVMDWARFLinker LLVMGlobalISel LLVMMIRParser LLVMAsmPrinter LLVMSelectionDAG LLVMCodeGen LLVMTarget LLVMObjCARCOpts LLVMCodeGenTypes LLVMIRPrinter LLVMInterfaceStub LLVMFileCheck LLVMFuzzMutate LLVMScalarOpts LLVMInstCombine LLVMAggressiveInstCombine LLVMTransformUtils LLVMBitWriter LLVMAnalysis LLVMProfileData LLVMSymbolize LLVMDebugInfoBTF LLVMDebugInfoPDB LLVMDebugInfoMSF LLVMDebugInfoDWARF LLVMObject LLVMTextAPI LLVMMCParser LLVMIRReader LLVMAsmParser LLVMMC LLVMDebugInfoCodeView LLVMBitReader LLVMFuzzerCLI LLVMCore LLVMRemarks LLVMBitstreamReader LLVMBinaryFormat LLVMTargetParser LLVMTableGen LLVMSupport LLVMDemangle OpenSSL::SSL OpenSSL::Crypto)
else()
target_link_libraries(neo2 PRIVATE LLVM OpenSSL::SSL OpenSSL::Crypto)
endif()
target_link_options(neo2 PRIVATE -Wl,--export-dynamic)
