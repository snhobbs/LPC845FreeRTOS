#------------------------------------------------------------------------------------------
# Set Project Defaults
#------------------------------------------------------------------------------------------
message("SDK:  ${SDK}")
if(NOT DEFINED SDK)
  message(fatal_error SDK is not defined)
endif()

if(NOT DEFINED toolchain_base)
  set(toolchain_base $ENV{HOME}/ToolChains)
endif()

if(NOT DEFINED CpplintCommand)
  set(CpplintCommand cpplint)
endif()

if(NOT DEFINED CppcheckCommand)
  set(CppcheckCommand cppcheck)
endif()

if(NOT DEFINED ClangtidyCommand)
  set(ClangtidyCommand clang-tidy)
endif()


#------------------------------------------------------------
# Add Project Sources
#------------------------------------------------------------
include(${LibraryDirectory}/CMakeStaticAnalysis/Utilities.cmake)
foreach (directory ${ProjectSourceDirectories})
    AUX_SOURCE_DIRECTORY(${directory} SourceFiles)
endforeach ()

foreach (file ${source_excludes})
  message("Remove ${file}")
  list(FILTER SourceFiles EXCLUDE REGEX ${file})
endforeach()

target_sources(${TargetName} PUBLIC ${SourceFiles})
get_target_property(SOURCES ${TargetName} TargetSources)
PrintSourceFiles("${TargetSources}")


#------------------------------------------------------------
# Setup Toolchain
#------------------------------------------------------------
include(${CMAKE_CURRENT_SOURCE_DIR}/arm-none-eabi_Toolchain.cmake)
if(NOT DEFINED BUILD_TYPE)
  set(BUILD_TYPE "GCC_ARM")
  message(WARNING "BUILD_TYPE not set, using ${BUILD_TYPE}")
endif()
string(TOUPPER "${BUILD_TYPE}" BUILD_TYPE)

if("${BUILD_TYPE}" STREQUAL MCUXPRESSO)
  set(LinkerScriptType "MCUXpresso")
else()
  set(LinkerScriptType "GCC_ARM")
endif()
message(STATUS "Using BUILD_TYPE ${BUILD_TYPE}")

if(NOT DEFINED LINKER_SCRIPT)
  if("${BUILD_TYPE}" STREQUAL MCUXPRESSO)
    set(LINKER_SCRIPT ${SDK}/devices/LPC845/mcuxpresso/LPC845_Project_Release.ld)
  else()
    set(LINKER_SCRIPT ${SDK}/devices/LPC845/gcc/LPC845_flash.ld)
  endif()
  message(WARNING "Linker script not set, using ${LINKER_SCRIPT}")
endif()

if(NOT EXISTS ${LINKER_SCRIPT})
  message(SEND_ERROR "Linker script ${LINKER_SCRIPT} not found")
endif()

if(NOT DEFINED LINKER_MAP)
  set(LINKER_MAP "${CMAKE_CURRENT_BINARY_DIR}/${TargetName}_${BUILD_TYPE}.map")
  # message(WARNING "Linker map not set, using ${LINKER_MAP}")
  message(STATUS "Linker Map: ${LINKER_MAP}")
endif()

SetupToolchain(${BUILD_TYPE} ${VendorCodeBaseDirectory} ${toolchain_base})
#  Sets the ArmStartupFile Option
target_sources(${TargetName} PUBLIC ${ArmStartupFile})

message(STATUS "C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Linker: ${CMAKE_CXX_LINKER}")

SetTargetOptions(${TargetName} ${LINKER_MAP} ${LINKER_SCRIPT} ${BUILD_TYPE})
set(TargetIncludes ${VendorDirectories} ${ProjectIncludeDirectories} ${DeviceLibrarySource})
SetTargetInclude("${TargetName}" "${TargetIncludes}")

