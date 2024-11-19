project(stm32l0xx-hal-driver)

file(GLOB_RECURSE STM32L0_HAL_SOURCES "stm32l0xx-hal-driver/Src/*.*")

# Remove template files from sources
set(STM32L0_HAL_SOURCES_CLEAN "")
foreach (SOURCE_FILE ${STM32L0_HAL_SOURCES})
    if (NOT SOURCE_FILE MATCHES "_template\.c$")
        list(APPEND STM32L0_HAL_SOURCES_CLEAN ${SOURCE_FILE})
    endif (NOT SOURCE_FILE MATCHES "_template\.c$")
endforeach (SOURCE_FILE ${STM32L0_HAL_SOURCES})

add_library(${PROJECT_NAME} STATIC ${STM32L0_HAL_SOURCES_CLEAN})
target_include_directories(${PROJECT_NAME} PUBLIC stm32l0xx-hal-driver/Inc)
target_include_directories(${PROJECT_NAME} PUBLIC cmsis-core/Core/Include)
target_include_directories(${PROJECT_NAME} PUBLIC cmsis-device-l0/Include)
