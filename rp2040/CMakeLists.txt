cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_SDK_PATH}/tools/CMakeLists.txt)

project(el_dirver C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release
RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

pico_sdk_init()

link_directories(${CMAKE_SOURCE_DIR}/lib)

add_executable(${PROJECT_NAME} 
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
)


AUX_SOURCE_DIRECTORY(lib SUB_SOURCES)

target_sources(
    ${PROJECT_NAME}
    PUBLIC
    ${SUB_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/lib
)

# set SPI clock slower than CPU clock so that it works
pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
pico_set_boot_stage2(${PROJECT_NAME} slower_boot2)


pico_generate_pio_header(${PROJECT_NAME}  
        ${CMAKE_CURRENT_LIST_DIR}/driver.pio
)

# pull in common dependencies
target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        pico_bootrom
        hardware_structs
        hardware_sync
        hardware_vreg
        hardware_clocks
        hardware_pio
        hardware_dma
        hardware_irq
        pico_multicore
)


# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})


# enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
