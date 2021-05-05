cmake_minimum_required(VERSION 3.10.0)

#------------------------------------------------------------------------------------------
# Project Settings
#------------------------------------------------------------------------------------------
set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_SUB 1)

set(CMAKE_BUILD_TYPE DEBUG)
set(HardwareName "LPC845MAX")
set(HardwareRevision "A")
project(${HardwareName}_${HardwareRevision}_FreeRTOSBlinky VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_SUB})
set(TargetName "${PROJECT_NAME}_V${VERSION_MAJOR}_${VERSION_MINOR}_${VERSION_SUB}_${CMAKE_BUILD_TYPE}")
message(STATUS "${PROJECT_NAME}")

set(CMAKE_BINARY_DIR "./")#${CMAKE_SOURCE_DIR})
set(ProjectDirectory ${CMAKE_CURRENT_SOURCE_DIR})
set(LibraryDirectory ${ProjectDirectory}/../libs)
set(ProjectSourceDirectories ${ProjectDirectory}/source)
set(ProjectIncludeDirectories ${ProjectSourceDirectories})
string(TIMESTAMP build_timestamp UTC)
#configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.h.in ${ProjectDirectories}/source/Config.h) # add header where version is passed to code


#------------------------------------------------------------------------------------------
# Project Settings
#------------------------------------------------------------------------------------------
add_executable(${TargetName})
set_property(TARGET ${TargetName} PROPERTY CXX_STANDARD 17)
set_property(TARGET ${TargetName} PROPERTY C_STANDARD 11)
set_property(TARGET ${TargetName} PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET ${TargetName} PROPERTY CMAKE_CXX_EXTENSIONS OFF)
set_property(TARGET ${TargetName} PROPERTY C_STANDARD_REQUIRED ON)
set_property(TARGET ${TargetName} PROPERTY CMAKE_C_EXTENSIONS OFF)

target_link_options(
  ${TargetName}
  PUBLIC
  $<$<CONFIG:RELEASE>:-s> # strip all symbols
  #-flto                          # Link time optimizations
  #-O3                            # Link time optimization level
  #-ffat-lto-objects              # Link time optimization in object files
)

target_compile_options(
    ${TargetName}
    PUBLIC
    $<$<CONFIG:Release>:-Os>
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Debug>:-g3>
    -Wfatal-errors
    -Wall
    -Wextra
    -pedantic
    -Wpedantic
    #-pedantic-errors
    #-Werror=pedantic

    -Wconversion
    $<$<COMPILE_LANGUAGE:C>:-Wimplicit>
    $<$<COMPILE_LANGUAGE:C>:-Wunused>

    $<$<CXX_COMPILER_ID:GNU>:-Wsuggest-final-types>
    $<$<CXX_COMPILER_ID:GNU>:-Wmaybe-uninitialized>
    $<$<CXX_COMPILER_ID:GNU>:-Wdouble-promotion>
    $<$<CXX_COMPILER_ID:GNU>:-Wnull-dereference>
    -Winit-self

    $<$<CXX_COMPILER_ID:GNU>:-Wduplicated-cond>
    $<$<CXX_COMPILER_ID:GNU>:-Wduplicated-branches>
    $<$<CXX_COMPILER_ID:GNU>:-Wlogical-op>
    $<$<CXX_COMPILER_ID:GNU>:-Wrestrict>
    $<$<C_COMPILER_ID:GNU>:$<$<COMPILE_LANGUAGE:C>:-Wjump-misses-init>>
    $<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>
    -Wshadow
)

set(source_excludes mtb.c semihost_hardfault.c uart.cpp)
#set(static_analysis_excludes ${source_excludes})
aux_source_directory(${ProjectDirectory}/source sources)

#------------------------------------------------------------
# Include Libraries
#------------------------------------------------------------

message("Library Path ${LibraryDirectory}")
set(FreeRTOS_DIR "${LibraryDirectory}/FreeRTOS/FreeRTOS/Source")
set(FreeRTOSPortable "${FreeRTOS_DIR}/portable/GCC/ARM_CM0")
#set(heap_implimentation "${FreeRTOS_DIR}/portable/MemMang/heap_2.c")
#set(heap_implimentation "${FreeRTOS_DIR}/portable/MemMang/heap_1.c")

target_include_directories(${TargetName} PUBLIC "${FreeRTOS_DIR}")
target_include_directories(${TargetName} PUBLIC "${FreeRTOS_DIR}/include")
target_include_directories(${TargetName} PUBLIC "${FreeRTOSPortable}")
aux_source_directory(${FreeRTOS_DIR} sources)
aux_source_directory(${FreeRTOSPortable} sources)
#list(APPEND static_analysis_excludes ${FreeeRTOS_DIR})

#====================================================
# Chip Specifics
#====================================================
set(ChipName LPC845)
include(${CMAKE_CURRENT_SOURCE_DIR}/Chip.cmake)
#foreach(dir ${VendorDirectories})

set(SDK ${LibraryDirectory}/SDK_2.8.0_LPC845)
set(VendorCodeBaseDirectory ${SDK})
include(${CMAKE_CURRENT_SOURCE_DIR}/ProjectBase.cmake)
SetupToolchain(${BUILD_TYPE} ${VendorCodeBaseDirectory} ${toolchain_base})
GetVendorDirectories("${VendorCodeBaseDirectory}")# Sets VendorDirectories
GetVendorSources("${VendorCodeBaseDirectory}") # Sets VendorSources
target_sources(${TargetName} PUBLIC ${VendorSources} ${sources} ${heap_implimentation})
message(${sources})
#foreach(dir VendorDirectories)
#  aux_source_directory(${dir} VendorSources)
#  message(${dir})
#endforeach()
target_include_directories(${TargetName} PUBLIC ${VendorDirectories})
#list(APPEND static_analysis_excludes ${VendorCodeBaseDirectories})

