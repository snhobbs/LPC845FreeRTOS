CMAKE_MINIMUM_REQUIRED (VERSION 3.10.0)
#include(CMakeForceCompiler)

# CMake Options
SET(CMAKE_VERBOSE_MAKEFILE ON)

# CROSS COMPILER SETTING
set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

# Clear flags to set in sane state
SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

set(CMAKE_C_IMPLICIT_LINK_DIRECTORIES)#THIS IS SUPER CRITICAL, KEEPS CMAKE FROM ADDING SYSTEM LIBRARIES
SET(CMAKE_STATIC_LIBRARY_LINK_CXX_FLAGS)#here on spec
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS)#Keeps CMake from adding rdynamic as a flag

set(CMAKE_EXECUTABLE_SUFFIX .axf)
set(TOOL_PREFIX arm-none-eabi)
set(TOOLCHAIN_EXT "")#no extension

# Compilers like arm-none-eabi-gcc that target bare metal systems don't pass
# CMake's compiler check, use the forcing functions and set explicitly
function(SetupGccToolChain toolchain_directory)
    SET(GCC_ARM_LINKER ${toolchain_directory}/${TOOL_PREFIX}-ld${TOOLCHAIN_EXT})
    SET(CMAKE_C_COMPILER ${toolchain_directory}/${TOOL_PREFIX}-gcc${TOOLCHAIN_EXT} PARENT_SCOPE)
    SET(CMAKE_CXX_COMPILER ${toolchain_directory}/${TOOL_PREFIX}-g++${TOOLCHAIN_EXT} PARENT_SCOPE)
    SET(CMAKE_ASM_COMPILER ${toolchain_directory}/${TOOL_PREFIX}-gcc${TOOLCHAIN_EXT} PARENT_SCOPE)
    SET(CMAKE_OBJCOPY ${toolchain_directory}/${TOOL_PREFIX}-objcopy PARENT_SCOPE)
    SET(CMAKE_OBJDUMP ${toolchain_directory}/${TOOL_PREFIX}-objdump PARENT_SCOPE)
endfunction()
   
function(SetupClangToolChain toolchain_directory)
    SET(GCC_ARM_LINKER ${toolchain_directory}/${TOOL_PREFIX}-ld${TOOLCHAIN_EXT})
    SET(CMAKE_C_COMPILER clang-9 PARENT_SCOPE)
    SET(CMAKE_CXX_COMPILER clang++-9 PARENT_SCOPE)
    SET(CMAKE_ASM_COMPILER clang-9 PARENT_SCOPE)
    SET(CMAKE_OBJCOPY llvm-objcopy-9 PARENT_SCOPE)
    SET(CMAKE_OBJDUMP llvm-objdump-9 PARENT_SCOPE)
    SET(CMAKE_LINKER ${GCC_ARM_LINKER} PARENT_SCOPE)
    SET(CMAKE_C_LINKER ${GCC_ARM_LINKER} PARENT_SCOPE)
    SET(CMAKE_CXX_LINKER ${GCC_ARM_LINKER} PARENT_SCOPE)
endfunction()
