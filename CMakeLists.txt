cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# ============================================================
# Compiler settings
# ============================================================

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# ============================================================
# Build type
# ============================================================

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# ============================================================
# Project
# ============================================================

set(CMAKE_PROJECT_NAME STM32_Cmake_Base)

# Enable compile commands for clangd / VS Code indexing
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})

message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable C and ASM
enable_language(C ASM)

# ============================================================
# Executable
# ============================================================

add_executable(${CMAKE_PROJECT_NAME})

# ============================================================
# Linker options
# ============================================================

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)

# ============================================================
# STM32CubeMX generated code
# ============================================================

add_subdirectory(cmake/stm32cubemx)

# ============================================================
# Hardware drivers
# ============================================================

file(GLOB HARDWARE_SOURCES
    CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/Hardware/Src/*.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${HARDWARE_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Hardware/Inc
)

# ============================================================
# Link directories
# ============================================================

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# ============================================================
# Include paths
# ============================================================

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# ============================================================
# Compile definitions
# ============================================================

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# ============================================================
# Remove wrong libob.a dependency when using C++
# ============================================================

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ============================================================
# Libraries
# ============================================================

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)