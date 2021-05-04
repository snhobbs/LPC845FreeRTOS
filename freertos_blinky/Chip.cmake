set(Triple thumbv6m-nxp-none-eabi)
set(MCPU cortex-m0plus)
set(float_abi soft)
set(cpu_identifier LPC845)
set(GCC_TOOLCHAIN gcc-arm-none-eabi-9-2019-q4-major/bin)
set(MCUXPRESSO_TOOLCHAIN com.nxp.mcuxpresso.tools.linux_11.3.0.202011031536/tools/bin)
if(NOT DEFINED CPU_TYPE)
  set(CPU_TYPE CPU_LPC845M301JHI33)
  message(WARNING "CPU_TYPE not set, using ${CPU_TYPE}")
endif()
    
# Exports: 
#   ArmStartupFile needs to be added to source
#   Compiler Settings
#
function(SetupToolchain BUILD_TYPE VendorCodeBaseDirectory toolchain_directory)
  message("Using ${BUILD_TYPE} toolchain")
  set(ArmCppStartupFile ${VendorCodeBaseDirectory}/devices/LPC845/mcuxpresso/startup_lpc845.cpp)
  set(ArmAssemblyStartupFile ${VendorCodeBaseDirectory}/devices/LPC845/gcc/startup_LPC845.S)
  if("${BUILD_TYPE}" STREQUAL MCUXPRESSO)
    set(ArmStartupFile ${ArmCppStartupFile} PARENT_SCOPE)
    SetupGccToolChain("${toolchain_directory}/${MCUXPRESSO_TOOLCHAIN}")
  elseif("${BUILD_TYPE}" STREQUAL GCC_ARM)
    set(ArmStartupFile ${ArmAssemblyStartupFile} PARENT_SCOPE)
    SetupGccToolChain("${toolchain_directory}/${GCC_TOOLCHAIN}")
    #message(SEND_ERROR "Unknown BUILD_TYPE")
  else()
    message(SEND_ERROR "Unknown BUILD_TYPE")
  endif()

  # Percolate changes up to the calling script
  SET(CMAKE_C_COMPILER ${CMAKE_C_COMPILER}  PARENT_SCOPE)
  SET(CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER}  PARENT_SCOPE)
  SET(CMAKE_ASM_COMPILER ${CMAKE_ASM_COMPILER}  PARENT_SCOPE)
  SET(CMAKE_OBJCOPY ${CMAKE_OBJCOPY} PARENT_SCOPE)
  SET(CMAKE_OBJDUMP ${CMAKE_OBJDUMP} PARENT_SCOPE)
  set(CMAKE_C_COMPILER_WORKS TRUE PARENT_SCOPE)
  set(CMAKE_CXX_COMPILER_WORKS TRUE PARENT_SCOPE)
  set(CMAKE_C_COMPILER_TARGET ${Triple} PARENT_SCOPE)
  set(CMAKE_CXX_COMPILER_TARGET ${Triple} PARENT_SCOPE)
  set(CMAKE_ASM_COMPILER_TARGET ${Triple} PARENT_SCOPE)
endfunction()

#
# Exports:
#   VendorDirectories -> List of chip directories
#
function(GetVendorDirectories device_directory)
    set(sub_directories devices/LPC845 devices/LPC845/drivers CMSIS CMSIS/Include) 
    set(vendor_directories)
    foreach (directory ${sub_directories})
        list(APPEND vendor_directories ${device_directory}/${directory})
    endforeach ()
    set(VendorDirectories ${vendor_directories} PARENT_SCOPE)
endfunction()

#
# Exports:
#   VendorSource -> All Source Files in the top lated of the
#           VendorDirectories
#
function(GetVendorSources device_directory)
    GetVendorDirectories(${device_directory})
    foreach(directory ${VendorDirectories})
        AUX_SOURCE_DIRECTORY(${directory} sources)
    endforeach()
    message("Vendor Sources " ${sources})
    set(VendorSources ${sources} PARENT_SCOPE)
endfunction()


function(SetupGccCompileOptions)
  target_compile_options(
    ${target}
    PUBLIC
    -fstrict-volatile-bitfields     # Access is controlled for memory mapped peripherals
    -mapcs-frame                    # Same as -mapcs Generate compliant stack frames (deprecated)
    -mapcs                          # Same as -mapcs-frame Generate compliant stack frames (deprecated)
    -mno-thumb-interwork            # Default setting, specifying anyway
    -mabort-on-noreturn                         # Calls abort if a function with no return tries to return
    )
endfunction()

function(SetupMCUXpressoCompileOptions target)
  target_compile_definitions(
    ${target}
    PUBLIC
    -D__NEWLIB__
    -D__MCUXPRESSO
    -D__USE_CMSIS
  )
endfunction()

function(SetupSharedCompileOptions target)
  target_compile_options(
    ${target}
    PUBLIC
    # Output Control
    #-c                                          # Do not link at this stage
    -v                                          # Print commands of each step

    # Preprocessor
    #-fanalyzer                                  # Run static analysis
    -ftabstop=2                                 # Set tabwidth for reporting column position
    #-MM
    #-MMD                                         
    #-MP

    # ASM Options

    #C Options
    $<$<COMPILE_LANGUAGE:ASM,C>:-ffreestanding> # implies -fno-builtin
    $<$<COMPILE_LANGUAGE:ASM,C>:-fno-builtin>       # Uses libc instead of compiler routines for somethings

    # C++ Options
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>        # Save some space when no dynamic_cast or exceptions are used
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>  # Reduces code size in none threaded code

    # Machine Specific Options
    -mcpu=${MCPU}                               # cpu type
    -mfloat-abi=${float_abi}                    # Software or hardware floats
    -mthumb                                     # Use Thumb instructions

    # Diagnostic Message Formatting
    -fmessage-length=0 
    
    # Optimization
    -ffunction-sections                         # Allows stripping of unused functions at link time with -gc-sections
    -fdata-sections                             # Allows stripping of unused variables at link time with -gc-sections 

    # Code Generation Options
    -fshort-enums                               # needed for clang compatibility
    -fno-common                                 # Specifies that the compiler places uninitialized global variables in the BSS section, needed for c++ compatibility
    -fno-exceptions                             # Turns off exceptions for languages that support it
  )
  target_compile_definitions(
    ${target}
    PUBLIC
    -DFSL_RTOS_BM 
    -DSDK_OS_BAREMETAL 
    -DSDK_DEBUGCONSOLE=0 
    -D${cpu_identifier}
    -D${CPU_TYPE}
    -D__STARTUP_CLEAR_BSS
    #-D__ATOLLIC__
  )
endfunction()

function(SetupGccLinkOptions target linker_map)
  target_link_options(
    ${target}
    PUBLIC
    $<$<C_COMPILER_ID:GNU>:-Wl,-Map=${linker_map}># Send option to linker: Linker map
    -Wl,--gc-sections               # remove unused sections of code
    -Wl,--sort-section=alignment    # Sort names by maximum alignment
    -Wl,-z -Wl,muldefs              # Send keyword to linker: Allow multiple definitions 
    -Wl,--no-export-dynamic         # Also keeps CMake from adding rdynamic as a flag
    $<$<C_COMPILER_ID:GNU>:-mcpu=${MCPU}> # GCC Architecuture type 
    $<$<C_COMPILER_ID:GNU>:-mthumb> # Use thumb architecture
    -T${linker_script}              # Linker script
    -static                         # Prevents linking with shared libraries
    -Wl,-print-memory-usage         # Print size of link sections after compilation

    #-nodefaultlibs                 # Need to have these
    #-nostartfiles                  # Need to have these
    #-nostdlib                      # Need to have these
    
    #$<$<C_COMPILER_ID:GNU>:--specs=nano.specs> # newlibnano library
    # $<$<C_COMPILER_ID:GNU>:--specs=nosys.specs> # nosemi hosting
  )
  target_link_libraries(
    ${TargetName}
    PUBLIC
    $<$<C_COMPILER_ID:GNU>:-Wl,--start-group>
    $<$<C_COMPILER_ID:GNU>:m>       # libm math library
    $<$<C_COMPILER_ID:GNU>:c>       # libc
    $<$<C_COMPILER_ID:GNU>:gcc>     # libgcc
    $<$<C_COMPILER_ID:GNU>:nosys>   # bare metal target
    $<$<C_COMPILER_ID:GNU>:-Wl,--end-group> 
    $<$<C_COMPILER_ID:GNU>:--specs=nano.specs> # newlibnano library
    $<$<C_COMPILER_ID:GNU>:--specs=nosys.specs> # nosemi hosting
  )
endfunction()

function(SetTargetOptions target linker_map linker_script BUILD_TYPE)
  SetupSharedCompileOptions(${target})
  if(${BUILD_TYPE} STREQUAL "GCC_ARM")
    SetupGccCompileOptions(${target})
    SetupGccLinkOptions(${target} ${linker_map})
  elseif(${BUILD_TYPE} STREQUAL "MCUXPRESSO")
    SetupMCUXpressoCompileOptions(${target})
    SetupGccLinkOptions(${target} ${linker_map})
  endif()
endfunction()

