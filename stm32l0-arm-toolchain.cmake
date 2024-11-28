# Specify the cross compiler
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(AS arm-none-eabi-as)
set(AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32L031K6Tx_FLASH_fix.ld)

set(COMMON_FLAGS "-mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections -specs=nosys.specs -specs=nano.specs -lnosys")

set(CMAKE_C_FLAGS "${COMMON_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections -T \"${LINKER_SCRIPT}\"")

add_definitions(
        -D__weak=__attribute__\(\(weak\)\)
        -D__packed=__attribute__\(\(__packed__\)\)
        -DUSE_HAL_DRIVER
        -DSTM32L031xx
        -DARM_MATH_CM0
)