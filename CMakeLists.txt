cmake_minimum_required(VERSION 3.10)

# Include toolchain file
include(stm32l0-arm-toolchain.cmake)

# Configure the project
project(firmware-probe C CXX ASM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 17)

# Add dependencies
add_subdirectory(External)

set(STARTUP_ASM "startup_stm32l031xx.s")

file(GLOB_RECURSE SOURCES
        "Core/Src/*.*"
        "Firmware/Src/*.*"
)

include_directories(
        Core/Inc
        Firmware/Inc
)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${STARTUP_ASM} ${LINKER_SCRIPT})
target_include_directories(${PROJECT_NAME}.elf PRIVATE "${PROJECT_BINARY_DIR}")
target_link_libraries(${PROJECT_NAME}.elf PRIVATE stm32l0xx-hal-driver)

# Pass HAL config to driver
target_include_directories(stm32l0xx-hal-driver PRIVATE Core/Inc)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(-D_DEBUG)
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=\"${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map\"")

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")

# Custom targets for interfacing with openocd
if (CMAKE_HOST_WIN32)
    add_custom_target(flash
            COMMAND openocd -f openocd.cfg -c init -c targets -c "reset init" -c "\"flash write_image erase \\\"${HEX_FILE}\\\"\"" -c "reset run" -c shutdown
            DEPENDS ${PROJECT_NAME}.elf
            COMMENT "Building and flashing ${PROJECT_NAME}.elf using openocd"
            SOURCES openocd.cfg
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
else()
    add_custom_target(flash
            COMMAND openocd -f openocd.cfg -c init -c targets -c "reset init" -c "flash write_image erase ${HEX_FILE}" -c "reset run" -c shutdown
            DEPENDS ${PROJECT_NAME}.elf
            COMMENT "Building and flashing ${PROJECT_NAME}.elf using openocd"
            SOURCES openocd.cfg
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif()

add_custom_target(flashonly
        COMMAND openocd -f openocd.cfg -c init -c targets -c "reset init" -c "flash write_image erase ${HEX_FILE}" -c "reset run" -c shutdown
        COMMENT "Flashing ${PROJECT_NAME}.elf using openocd"
        SOURCES openocd.cfg
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

add_custom_target(debug
        COMMAND openocd -f openocd.cfg -c init -c targets -c "reset halt"
        COMMENT "Running openocd debug"
        SOURCES openocd.cfg
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)