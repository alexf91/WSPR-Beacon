cmake_minimum_required(VERSION 3.0)

project(wspr-beacon C ASM)

set(MCU "atmega328p")
set(F_CPU 16000000)

# Use avr-gcc
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_ASM_COMPILER avr-gcc)

# Defines used in the project
add_definitions(-D F_CPU=${F_CPU})

# Remove the -rdynamic flag
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")

# Set the compiler flags
set(CMAKE_C_FLAGS "-mmcu=${MCU} -Os")
set(CMAKE_ASM_FLAGS ${CMAKE_C_FLAGS})


add_executable(${PROJECT_NAME}
    beacon.c
    wspr.c
    wspr.h

    # Internal hardware
    hal/i2c_master.c
    hal/i2c_master.h
    hal/si5351.c
    hal/si5351.h
    hal/uart.c
    hal/uart.h

    # USB interface
    vusb/asmcommon.inc
    vusb/oddebug.c
    vusb/oddebug.h
    vusb/usbconfig.h
    vusb/usbdrvasm16.inc
    vusb/usbdrvasm.S
    vusb/usbdrv.c
    vusb/usbdrv.h
    vusb/usbportability.h
)

set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")
include_directories(${PROJECT_NAME} hal vusb)


# Target for flashing
add_custom_target(flash
    COMMAND avrdude -p ${MCU} -P usb -c avrispmkii -e -U flash:w:${PROJECT_NAME}.elf
    DEPENDS ${PROJECT_NAME}
)

add_custom_target(size
    COMMAND avr-size -C --mcu ${MCU} ${PROJECT_NAME}.elf
    DEPENDS ${PROJECT_NAME}
)