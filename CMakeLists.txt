#
# CMakeLists.txt
#
#  Created on: 24 dec. 2025
#      Author: Ludo
#

# Minimum CMake version.
cmake_minimum_required(VERSION 3.23)

# Project creation.
project(sx1232-driver)

# Build mode.
if(NOT DEFINED BUILD_MODE)
    set(BUILD_MODE "STATIC")
endif()

# Create library.
add_library(${PROJECT_NAME} ${BUILD_MODE})

# Static library.
if(${BUILD_MODE} STREQUAL STATIC)

    # Macro to convert options to variables.
    macro(add_compilation_flag FLAG_NAME FLAG_DESCRIPTION FLAG_DEFAULT_VALUE)
        option(FLAG_NAME FLAG_DESCRIPTION FLAG_DEFAULT_VALUE)
        set(${FLAG_NAME} ${FLAG_DEFAULT_VALUE} CACHE STRING ${FLAG_DESCRIPTION})
        list(APPEND COMPILATION_FLAGS_LIST ${FLAG_NAME})
    endmacro()
    
    # Compilation flags.
    add_compilation_flag(SX1232_DRIVERS_DISABLE "Disable the SX1232 driver." OFF)
    add_compilation_flag(SX1232_DRIVER_SPI_ERROR_BASE_LAST "Last error base of the low level SPI driver." 0)
    add_compilation_flag(SX1232_DRIVER_DELAY_ERROR_BASE_LAST "Last error base of the low level delay driver." 0)
    add_compilation_flag(SX1232_DRIVER_FXOSC_HZ "Oscillator frequency in Hz." 32000000)
    add_compilation_flag(SX1232_DRIVER_TX_ENABLE "Enable radio transmission functions." ON)
    add_compilation_flag(SX1232_DRIVER_RX_ENABLE "Enable radio reception functions." ON)
    
    # Remove OFF flags from list and keep flags set to value 0.
    foreach(FLAG ${COMPILATION_FLAGS_LIST})
        if((NOT ${FLAG} STREQUAL OFF) AND (NOT ${FLAG} STREQUAL ON))
            set(${FLAG} "(${${FLAG}})")
        endif()
    endforeach()   
    
    # Create flags file.
    configure_file(sx1232_driver_flags.h.in sx1232_driver_flags.h @ONLY)
    
    # Fixed compilation flags of dependencies.
    if(${BUILD_MODE} STREQUAL STATIC)
        target_compile_definitions(${PROJECT_NAME}
            PRIVATE
                EMBEDDED_UTILS_DISABLE_FLAGS_FILE
        )
    endif()
    
    # Dependencies folders.
    target_include_directories(${PROJECT_NAME}
        PUBLIC
            ${CMAKE_CURRENT_BINARY_DIR}
            ${TYPES_PATH}
            ${EMBEDDED_UTILS_PATH}/inc
    )
    
    # Print archive size.
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD 
        COMMAND ${CMAKE_SIZE_UTIL} -t lib${PROJECT_NAME}.a
    )
    
endif()

# Source files list.
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sx1232_hw.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sx1232.c
)

# Header files folder.
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)